#include "ServoBus.h"

#include <driver/twai.h>

// -----------------------------------------------------------------------------
//  State — all file-static, written on the protocol task, read via snapshot
// -----------------------------------------------------------------------------

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static bool s_initialized = false;

// Config (written by UI task, single-word writes)
static volatile uint8_t  s_addr         = asmg::kAddrFactoryDefault;
static volatile bool     s_bigEndian    = true;    // matches codec default assumption
static volatile bool     s_polling      = false;   // off at boot — bring-up starts manual
static volatile uint8_t  s_pollCmd      = 0x07;
static volatile uint16_t s_pollMs       = 20;      // 50 Hz
static volatile bool     s_serialMirror = false;

// One-shot request mailbox (UI sets, protocol task clears)
static volatile bool     s_reqReadId  = false;
static volatile bool     s_reqReadPid = false;
static volatile bool     s_reqReadCfg = false;
static volatile bool     s_reqClear   = false;
static volatile bool     s_posPending = false;
static volatile uint16_t s_posCounts  = 0;
static volatile uint16_t s_posSpeed   = 0;

// Outstanding request tracking — strictly one at a time
static bool     s_waiting    = false;
static uint8_t  s_waitCmd    = 0;
static uint8_t  s_waitAddr   = 0;
static uint32_t s_sentUs     = 0;
static uint32_t s_lastPollMs = 0;
static const uint32_t kReplyTimeoutMs = 50;   // generous for diagnosis; brief targets 20 ms

static ServoSnapshot s_snap = {};

// Trace ring
static const uint32_t kTraceSize = 128;
static TraceEntry s_trace[kTraceSize];
static uint32_t   s_traceSeq = 0;             // seq of the NEXT entry to be written

// -----------------------------------------------------------------------------
//  Helpers
// -----------------------------------------------------------------------------

// Runtime-switchable word order. The codec's packWord/unpackWord are fixed at
// compile time; the sniffer's job is to discover which order is right, so it
// packs and parses with this flag instead. Once decided, bake the answer into
// ASMG_WORD_BIG_ENDIAN in asmg_md_can.h.
static void pack16(uint8_t* p, uint16_t v) {
    if (s_bigEndian) { p[0] = v >> 8;   p[1] = v & 0xFF; }
    else             { p[0] = v & 0xFF; p[1] = v >> 8;   }
}
static uint16_t unpack16(const uint8_t* p) {
    return s_bigEndian ? (uint16_t)((p[0] << 8) | p[1])
                       : (uint16_t)((p[1] << 8) | p[0]);
}

static void tracePush(uint8_t dir, uint32_t id, uint8_t dlc, const uint8_t* data) {
    portENTER_CRITICAL(&s_mux);
    TraceEntry& e = s_trace[s_traceSeq % kTraceSize];
    e.seq = s_traceSeq++;
    e.ms  = millis();
    e.id  = id;
    e.dlc = dlc;
    memcpy(e.data, data, 8);
    e.dir = dir;
    portEXIT_CRITICAL(&s_mux);

    if (s_serialMirror) {
        static const char* dirTag[] = { "RX", "TX", "TX-FAIL" };
        Serial.printf("%8lu %-7s %08lX ", (unsigned long)millis(),
                      dirTag[dir], (unsigned long)id);
        for (int i = 0; i < 8; i++) Serial.printf("%02X ", data[i]);
        Serial.println();
    }
}

// Send a frame; returns true if queued OK. Single-shot (no retransmit) so a
// missing servo shows up as TX-FAIL instead of an error-frame retry storm —
// the same storm that disturbs this panel's PSRAM DMA in the main firmware.
static bool sendFrame(const asmg::Frame& f) {
    twai_message_t msg = {};
    msg.extd             = 1;
    msg.ss               = 1;
    msg.identifier       = f.id;
    msg.data_length_code = f.dlc;
    memcpy(msg.data, f.data, 8);

    bool ok = twai_transmit(&msg, 0) == ESP_OK;
    tracePush(ok ? TRACE_TX : TRACE_TX_FAIL, f.id, f.dlc, f.data);
    portENTER_CRITICAL(&s_mux);
    if (ok) s_snap.txCount++; else s_snap.txFails++;
    portEXIT_CRITICAL(&s_mux);
    return ok;
}

static bool installDriver() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)SERVO_CAN_TX_PIN, (gpio_num_t)SERVO_CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    return twai_driver_install(&g, &t, &f) == ESP_OK && twai_start() == ESP_OK;
}

static void checkHealth() {
    twai_status_info_t info;
    if (twai_get_status_info(&info) != ESP_OK) return;

    portENTER_CRITICAL(&s_mux);
    s_snap.busState = (uint8_t)info.state;
    s_snap.busErrTx = info.tx_error_counter;
    s_snap.busErrRx = info.rx_error_counter;
    portEXIT_CRITICAL(&s_mux);

    if (info.state == TWAI_STATE_BUS_OFF || info.state == TWAI_STATE_RECOVERING ||
        info.state == TWAI_STATE_STOPPED) {
        Serial.printf("SNIFF: TWAI state=%d — reinstalling\n", info.state);
        twai_stop();
        twai_driver_uninstall();
        delay(50);
        s_initialized = installDriver();
        portENTER_CRITICAL(&s_mux);
        s_snap.busRecoveries++;
        portEXIT_CRITICAL(&s_mux);
        Serial.println(s_initialized ? "SNIFF: recovery OK" : "SNIFF: recovery FAILED");
    }
}

// Parse a frame that arrived while a request was outstanding.
static void handleReply(const twai_message_t& msg) {
    const uint8_t* d = msg.data;
    uint32_t nowUs = micros();

    portENTER_CRITICAL(&s_mux);
    s_snap.lastAnyRxId = msg.identifier;
    s_snap.rxCount++;
    portEXIT_CRITICAL(&s_mux);

    if (!s_waiting) return;   // unsolicited — visible in the trace, not decoded

    // Address check: 0xFD was sent broadcast, so byte 0 is the discovery
    // answer, not an echo of what we polled. Everything else must echo addr.
    bool addrOk = (s_waitCmd == (uint8_t)asmg::Cmd::ReadId) || (d[0] == s_waitAddr);
    // The 0x04 reply may echo 0x03 per the manufacturer doc — accept both and
    // record which one actually arrived (bring-up step 8).
    bool cmdOk = (d[1] == s_waitCmd) ||
                 (s_waitCmd == (uint8_t)asmg::Cmd::ReadCurrentConfig &&
                  d[1] == (uint8_t)asmg::Cmd::SetCurrent);
    if (!addrOk || !cmdOk) return;

    uint32_t lat = nowUs - s_sentUs;

    portENTER_CRITICAL(&s_mux);
    s_snap.latLastUs = lat;
    if (lat > s_snap.latMaxUs) s_snap.latMaxUs = lat;
    s_snap.lastGoodMs = millis();

    switch (s_waitCmd) {
        case 0x02:
            s_snap.position    = unpack16(&d[2]);
            s_snap.cmdPosition = unpack16(&d[4]);
            s_snap.havePos = s_snap.haveCmdPos = true;
            break;
        case 0x04:
            s_snap.torque        = unpack16(&d[2]);
            s_snap.currentLimit  = unpack16(&d[4]);
            s_snap.replyEchoCmd04 = d[1];
            s_snap.haveCfg = true;
            break;
        case 0x06:
            s_snap.pidP = unpack16(&d[2]);
            s_snap.pidI = unpack16(&d[4]);
            s_snap.pidD = unpack16(&d[6]);
            s_snap.havePid = true;
            break;
        case 0x07:
            s_snap.position = unpack16(&d[2]);
            s_snap.current  = unpack16(&d[4]);
            s_snap.havePos = s_snap.haveCur = true;
            break;
        case 0xFD:
            s_snap.discoveredAddr = d[0];   // provisional — see parseReadId note
            s_snap.haveAddr = true;
            break;
    }
    portEXIT_CRITICAL(&s_mux);

    s_waiting = false;
}

// -----------------------------------------------------------------------------
//  Public API
// -----------------------------------------------------------------------------

void servoBusBegin() {
    s_initialized = installDriver();
    Serial.printf("SNIFF: TWAI %s — 250 kbit/s, TX=%d RX=%d, filter open\n",
                  s_initialized ? "up" : "INIT FAILED",
                  SERVO_CAN_TX_PIN, SERVO_CAN_RX_PIN);
}

void servoBusUpdate() {
    if (!s_initialized) return;

    uint32_t now = millis();

    static uint32_t lastHealth = 0;
    if (now - lastHealth >= 1000) { checkHealth(); lastHealth = now; }

    // Drain RX
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        tracePush(TRACE_RX, msg.identifier, msg.data_length_code, msg.data);
        handleReply(msg);
    }

    // Reply timeout
    if (s_waiting && (micros() - s_sentUs) / 1000 >= kReplyTimeoutMs) {
        s_waiting = false;
        portENTER_CRITICAL(&s_mux);
        s_snap.timeouts++;
        portEXIT_CRITICAL(&s_mux);
    }

    if (s_reqClear) {
        s_reqClear = false;
        portENTER_CRITICAL(&s_mux);
        s_snap.timeouts = s_snap.txFails = 0;
        s_snap.latLastUs = s_snap.latMaxUs = 0;
        s_snap.rxCount = s_snap.txCount = 0;
        portEXIT_CRITICAL(&s_mux);
    }

    if (s_waiting) return;    // strictly one outstanding request

    // Position command first: no reply expected, fire and stay idle
    if (s_posPending) {
        s_posPending = false;
        asmg::Frame f = asmg::makeFrame(s_addr, asmg::Cmd::SetPositionSpeed);
        uint16_t counts = s_posCounts > asmg::kPositionMax ? asmg::kPositionMax
                                                           : s_posCounts;
        pack16(&f.data[2], counts);
        pack16(&f.data[4], s_posSpeed);
        sendFrame(f);
        return;               // give the bus a tick before the next request
    }

    // One-shot reads take priority over the poll loop
    uint8_t oneShot = 0;
    if      (s_reqReadId)  { s_reqReadId  = false; oneShot = 0xFD; }
    else if (s_reqReadPid) { s_reqReadPid = false; oneShot = 0x06; }
    else if (s_reqReadCfg) { s_reqReadCfg = false; oneShot = 0x04; }

    if (oneShot) {
        asmg::Frame f = (oneShot == 0xFD)
            ? asmg::cmdReadId()
            : asmg::makeFrame(s_addr, (asmg::Cmd)oneShot);
        if (sendFrame(f)) {
            s_waiting  = true;
            s_waitCmd  = oneShot;
            s_waitAddr = s_addr;
            s_sentUs   = micros();
        }
        return;
    }

    // Steady-state poll
    if (s_polling && now - s_lastPollMs >= s_pollMs) {
        s_lastPollMs = now;
        asmg::Frame f = asmg::makeFrame(s_addr, (asmg::Cmd)(uint8_t)s_pollCmd);
        if (sendFrame(f)) {
            s_waiting  = true;
            s_waitCmd  = s_pollCmd;
            s_waitAddr = s_addr;
            s_sentUs   = micros();
        }
    }
}

ServoSnapshot servoGetSnapshot() {
    ServoSnapshot s;
    portENTER_CRITICAL(&s_mux);
    s = s_snap;
    portEXIT_CRITICAL(&s_mux);
    s.replyFresh = s.lastGoodMs && (millis() - s.lastGoodMs < 500);
    return s;
}

uint32_t servoTraceRead(uint32_t* lastSeq, TraceEntry* out, uint32_t maxOut) {
    uint32_t n = 0;
    portENTER_CRITICAL(&s_mux);
    uint32_t newest = s_traceSeq;
    uint32_t from   = *lastSeq;
    // If the reader fell more than a ring behind, skip to the oldest retained
    if (newest - from > kTraceSize) from = newest - kTraceSize;
    while (from < newest && n < maxOut) {
        out[n++] = s_trace[from % kTraceSize];
        from++;
    }
    portEXIT_CRITICAL(&s_mux);
    *lastSeq = from;
    return n;
}

void servoSetAddress(uint8_t addr)        { s_addr = addr; }
uint8_t servoGetAddress()                 { return s_addr; }
void servoSetBigEndian(bool be)           { s_bigEndian = be; }
bool servoGetBigEndian()                  { return s_bigEndian; }
void servoSetPolling(bool on)             { s_polling = on; }
bool servoGetPolling()                    { return s_polling; }
void servoSetPollCmd(uint8_t cmd)         { s_pollCmd = cmd; }
void servoSetPollIntervalMs(uint16_t ms)  { s_pollMs = ms < 5 ? 5 : ms; }
void servoSetSerialMirror(bool on)        { s_serialMirror = on; }
bool servoGetSerialMirror()               { return s_serialMirror; }

void servoCommandPosition(uint16_t counts, uint16_t speed) {
    s_posCounts = counts;
    s_posSpeed  = speed;
    s_posPending = true;
}
void servoRequestReadId()  { s_reqReadId = true; }
void servoRequestReadPid() { s_reqReadPid = true; }
void servoRequestReadCfg() { s_reqReadCfg = true; }
void servoClearCounters()  { s_reqClear = true; }
