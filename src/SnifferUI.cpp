// =============================================================================
//  SnifferUI — 800x480 single-screen diagnostic UI for the ASMG-MD servo
// =============================================================================
//  Everything runs on the LVGL task. A 100 ms lv_timer pulls a ServoSnapshot
//  and drains the frame trace; control widgets call the thread-safe setters in
//  ServoBus.h. No destructive servo command is reachable from this UI.
// =============================================================================
#include "SnifferUI.h"

#include <Arduino.h>
#include <lvgl.h>
#include "ServoBus.h"

// --- palette ---
#define COL_BG      lv_color_hex(0x0E1116)
#define COL_PANEL   lv_color_hex(0x161B22)
#define COL_TEXT    lv_color_hex(0xE6EDF3)
#define COL_MUTED   lv_color_hex(0x8B949E)
#define COL_ACCENT  lv_color_hex(0x2AA8D0)
#define COL_GOOD    lv_color_hex(0x3FB950)
#define COL_WARN    lv_color_hex(0xD29922)
#define COL_BAD     lv_color_hex(0xF85149)

// --- widgets we update at runtime ---
static lv_obj_t* s_statusDot;
static lv_obj_t* s_statusLbl;
static lv_obj_t* s_rxIdLbl;
static lv_obj_t* s_latLbl;
static lv_obj_t* s_cntLbl;

static lv_obj_t* s_posVal;
static lv_obj_t* s_posSub;
static lv_obj_t* s_curVal;
static lv_obj_t* s_curSub;
static lv_obj_t* s_cmdVal;
static lv_obj_t* s_cmdSub;
static lv_obj_t* s_cfgLbl;

static lv_obj_t* s_slider;
static lv_obj_t* s_targetLbl;
static lv_obj_t* s_liveSw;
static lv_obj_t* s_speedDd;

static lv_obj_t* s_limMinLbl;
static lv_obj_t* s_limMaxLbl;

static lv_obj_t* s_addrLbl;
static lv_obj_t* s_adoptBtnLbl;
static lv_obj_t* s_traceLbl;
static lv_obj_t* s_pauseBtnLbl;

// --- UI-side state ---
static uint16_t s_speedVals[4] = { 0x0000, 0x0100, 0x0300, 0x0500 };
static uint16_t s_limMin = 0,      s_limMax = asmg::kPositionMax;
static bool     s_limMinSet = false, s_limMaxSet = false;
static uint8_t  s_uiAddr = 0;
static bool     s_tracePaused = false;
static uint32_t s_traceSeq = 0;

#define TRACE_LINES 14
static char s_traceBuf[TRACE_LINES][44];
static int  s_traceHead = 0;
static int  s_traceCount = 0;

// -----------------------------------------------------------------------------
//  Actions
// -----------------------------------------------------------------------------

static uint16_t selectedSpeed() {
    return s_speedVals[lv_dropdown_get_selected(s_speedDd)];
}

static void sendTarget(uint16_t counts) {
    if (counts < s_limMin) counts = s_limMin;
    if (counts > s_limMax) counts = s_limMax;
    servoCommandPosition(counts, selectedSpeed());
}

static void applySliderRange() {
    uint16_t lo = s_limMin, hi = s_limMax;
    if (lo >= hi) { lo = 0; hi = asmg::kPositionMax; }   // nonsense capture — ignore
    lv_slider_set_range(s_slider, lo, hi);
    lv_label_set_text_fmt(s_limMinLbl, "MIN %s%u",  s_limMinSet ? "" : "~", lo);
    lv_label_set_text_fmt(s_limMaxLbl, "MAX %s%u",  s_limMaxSet ? "" : "~", hi);
}

// -----------------------------------------------------------------------------
//  Event callbacks
// -----------------------------------------------------------------------------

static void onSlider(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    uint16_t v = (uint16_t)lv_slider_get_value(s_slider);
    lv_label_set_text_fmt(s_targetLbl, "%u", v);

    if (code == LV_EVENT_RELEASED) {
        sendTarget(v);
    } else if (code == LV_EVENT_VALUE_CHANGED && lv_obj_has_state(s_liveSw, LV_STATE_CHECKED)) {
        static uint32_t lastSend = 0;
        if (millis() - lastSend >= 100) {         // live drag: max 10 Hz
            lastSend = millis();
            sendTarget(v);
        }
    }
}

static void onNudge(lv_event_t* e) {
    int32_t delta = (int32_t)(intptr_t)lv_event_get_user_data(e);
    int32_t v = lv_slider_get_value(s_slider) + delta;
    int32_t lo = lv_slider_get_min_value(s_slider);
    int32_t hi = lv_slider_get_max_value(s_slider);
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    lv_slider_set_value(s_slider, v, LV_ANIM_OFF);
    lv_label_set_text_fmt(s_targetLbl, "%ld", (long)v);
    sendTarget((uint16_t)v);
}

static void onSetMin(lv_event_t*) {
    ServoSnapshot s = servoGetSnapshot();
    s_limMin = s.havePos ? s.position : (uint16_t)lv_slider_get_value(s_slider);
    s_limMinSet = true;
    applySliderRange();
}
static void onSetMax(lv_event_t*) {
    ServoSnapshot s = servoGetSnapshot();
    s_limMax = s.havePos ? s.position : (uint16_t)lv_slider_get_value(s_slider);
    s_limMaxSet = true;
    applySliderRange();
}
static void onClearLimits(lv_event_t*) {
    s_limMin = 0; s_limMax = asmg::kPositionMax;
    s_limMinSet = s_limMaxSet = false;
    applySliderRange();
}

static void onPollSw(lv_event_t* e) {
    servoSetPolling(lv_obj_has_state((lv_obj_t*)lv_event_get_target(e), LV_STATE_CHECKED));
}
static void onRateDd(lv_event_t* e) {
    static const uint16_t ms[] = { 10, 20, 50, 100, 200 };
    servoSetPollIntervalMs(ms[lv_dropdown_get_selected((lv_obj_t*)lv_event_get_target(e))]);
}
static void onCmdDd(lv_event_t* e) {
    servoSetPollCmd(lv_dropdown_get_selected((lv_obj_t*)lv_event_get_target(e)) == 0 ? 0x07 : 0x02);
}
static void onEndianSw(lv_event_t* e) {
    // checked = little-endian (the non-default hypothesis)
    servoSetBigEndian(!lv_obj_has_state((lv_obj_t*)lv_event_get_target(e), LV_STATE_CHECKED));
}
static void onSerialSw(lv_event_t* e) {
    servoSetSerialMirror(lv_obj_has_state((lv_obj_t*)lv_event_get_target(e), LV_STATE_CHECKED));
}

static void setUiAddr(uint8_t a) {
    s_uiAddr = a;
    servoSetAddress(a);
    lv_label_set_text_fmt(s_addrLbl, "0x%02X", a);
}
static void onAddrDelta(lv_event_t* e) {
    int d = (int)(intptr_t)lv_event_get_user_data(e);
    int a = (int)s_uiAddr + d;
    if (a < 0) a = 0;
    if (a > 0xFD) a = 0xFD;    // 0xFE is broadcast, never a unit address
    setUiAddr((uint8_t)a);
}
static void onAdoptAddr(lv_event_t*) {
    ServoSnapshot s = servoGetSnapshot();
    if (s.haveAddr) setUiAddr(s.discoveredAddr);
}

static void onReadId(lv_event_t*)  { servoRequestReadId();  }
static void onReadPid(lv_event_t*) { servoRequestReadPid(); }
static void onReadCfg(lv_event_t*) { servoRequestReadCfg(); }
static void onClrCnt(lv_event_t*)  { servoClearCounters();  }

static void onPause(lv_event_t*) {
    s_tracePaused = !s_tracePaused;
    lv_label_set_text(s_pauseBtnLbl, s_tracePaused ? "RUN" : "PAUSE");
}

// -----------------------------------------------------------------------------
//  Periodic refresh
// -----------------------------------------------------------------------------

static void refreshTrace() {
    TraceEntry ent[16];
    uint32_t n = servoTraceRead(&s_traceSeq, ent, 16);
    if (s_tracePaused) return;                    // seq still advances = catch-up on resume
    for (uint32_t i = 0; i < n; i++) {
        const TraceEntry& t = ent[i];
        char* line = s_traceBuf[s_traceHead];
        char dir = t.dir == TRACE_RX ? '<' : (t.dir == TRACE_TX ? '>' : 'X');
        snprintf(line, sizeof(s_traceBuf[0]),
                 "%05lu %c %08lX %02X%02X %02X%02X %02X%02X %02X%02X",
                 (unsigned long)(t.ms % 100000), dir, (unsigned long)t.id,
                 t.data[0], t.data[1], t.data[2], t.data[3],
                 t.data[4], t.data[5], t.data[6], t.data[7]);
        s_traceHead = (s_traceHead + 1) % TRACE_LINES;
        if (s_traceCount < TRACE_LINES) s_traceCount++;
    }
    if (n == 0) return;

    static char text[TRACE_LINES * 44];
    char* p = text;
    for (int i = 0; i < s_traceCount; i++) {
        int idx = (s_traceHead - s_traceCount + i + TRACE_LINES) % TRACE_LINES;
        p += snprintf(p, sizeof(text) - (p - text), "%s\n", s_traceBuf[idx]);
    }
    lv_label_set_text(s_traceLbl, text);
}

static void refreshTick(lv_timer_t*) {
    ServoSnapshot s = servoGetSnapshot();

    // Status bar
    lv_color_t dot;
    const char* st;
    if (s.busState != 1 /*TWAI_STATE_RUNNING*/) { dot = COL_BAD;  st = "BUS ERR"; }
    else if (s.replyFresh)                      { dot = COL_GOOD; st = "ONLINE";  }
    else if (servoGetPolling())                 { dot = COL_BAD;  st = "NO REPLY";}
    else                                        { dot = COL_MUTED;st = "IDLE";    }
    lv_obj_set_style_bg_color(s_statusDot, dot, 0);
    lv_label_set_text(s_statusLbl, st);

    if (s.lastAnyRxId) lv_label_set_text_fmt(s_rxIdLbl, "RX ID %08lX", (unsigned long)s.lastAnyRxId);
    lv_label_set_text_fmt(s_latLbl, "lat %lu.%lu / %lu.%lums",
                          (unsigned long)(s.latLastUs / 1000), (unsigned long)((s.latLastUs % 1000) / 100),
                          (unsigned long)(s.latMaxUs  / 1000), (unsigned long)((s.latMaxUs  % 1000) / 100));
    lv_label_set_text_fmt(s_cntLbl, "TO %lu  TXF %lu  E %lu/%lu",
                          (unsigned long)s.timeouts, (unsigned long)s.txFails,
                          (unsigned long)s.busErrTx, (unsigned long)s.busErrRx);

    // Value tiles
    if (s.havePos) {
        lv_label_set_text_fmt(s_posVal, "%u", s.position);
        lv_label_set_text_fmt(s_posSub, "0x%04X  %u.%u%%", s.position,
                              s.position * 100u / asmg::kPositionMax,
                              (s.position * 1000u / asmg::kPositionMax) % 10);
    }
    if (s.haveCur) {
        lv_label_set_text_fmt(s_curVal, "%u", s.current);
        lv_label_set_text_fmt(s_curSub, "0x%04X raw", s.current);
    }
    if (s.haveCmdPos) {
        lv_label_set_text_fmt(s_cmdVal, "%u", s.cmdPosition);
        lv_label_set_text_fmt(s_cmdSub, "0x%04X", s.cmdPosition);
    }

    // PID / current-config line
    char pid[20] = "-/-/-", cfg[28] = "LIM -  TRQ -";
    if (s.havePid) snprintf(pid, sizeof(pid), "%u/%u/%u", s.pidP, s.pidI, s.pidD);
    if (s.haveCfg) snprintf(cfg, sizeof(cfg), "LIM %u  TRQ %u  (echo %02X)",
                            s.currentLimit, s.torque, s.replyEchoCmd04);
    lv_label_set_text_fmt(s_cfgLbl, "PID %s    %s", pid, cfg);

    // 0xFD result
    if (s.haveAddr)
        lv_label_set_text_fmt(s_adoptBtnLbl, "ADOPT %02X", s.discoveredAddr);

    refreshTrace();
}

// -----------------------------------------------------------------------------
//  Widget factory helpers
// -----------------------------------------------------------------------------

static lv_obj_t* mkLabel(lv_obj_t* parent, const char* txt, const lv_font_t* font,
                         lv_color_t color, int x, int y) {
    lv_obj_t* l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, color, 0);
    lv_obj_set_pos(l, x, y);
    return l;
}

static lv_obj_t* mkPanel(lv_obj_t* parent, int x, int y, int w, int h) {
    lv_obj_t* p = lv_obj_create(parent);
    lv_obj_set_pos(p, x, y);
    lv_obj_set_size(p, w, h);
    lv_obj_set_style_bg_color(p, COL_PANEL, 0);
    lv_obj_set_style_border_width(p, 0, 0);
    lv_obj_set_style_radius(p, 8, 0);
    lv_obj_set_style_pad_all(p, 6, 0);
    lv_obj_remove_flag(p, LV_OBJ_FLAG_SCROLLABLE);
    return p;
}

static lv_obj_t* mkBtn(lv_obj_t* parent, const char* txt, int x, int y, int w, int h,
                       lv_event_cb_t cb, void* user, lv_obj_t** lblOut = nullptr) {
    lv_obj_t* b = lv_button_create(parent);
    lv_obj_set_pos(b, x, y);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x21303D), 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, user);
    lv_obj_t* l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_center(l);
    if (lblOut) *lblOut = l;
    return b;
}

static lv_obj_t* mkSwitch(lv_obj_t* parent, int x, int y, lv_event_cb_t cb) {
    lv_obj_t* sw = lv_switch_create(parent);
    lv_obj_set_pos(sw, x, y);
    lv_obj_set_size(sw, 50, 26);
    lv_obj_add_event_cb(sw, cb, LV_EVENT_VALUE_CHANGED, nullptr);
    return sw;
}

// -----------------------------------------------------------------------------
//  Screen construction
// -----------------------------------------------------------------------------

void snifferUiInit() {
    lv_obj_t* scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, COL_BG, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // ---- status bar ----
    mkLabel(scr, "ASMG SNIFF", &lv_font_montserrat_20, COL_ACCENT, 8, 10);
    s_statusDot = lv_obj_create(scr);
    lv_obj_set_pos(s_statusDot, 140, 12);
    lv_obj_set_size(s_statusDot, 16, 16);
    lv_obj_set_style_radius(s_statusDot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_statusDot, COL_MUTED, 0);
    lv_obj_set_style_border_width(s_statusDot, 0, 0);
    s_statusLbl = mkLabel(scr, "IDLE",         &lv_font_montserrat_14, COL_TEXT,  164, 13);
    s_rxIdLbl   = mkLabel(scr, "RX ID --------",&lv_font_montserrat_14, COL_TEXT, 268, 13);
    s_latLbl    = mkLabel(scr, "lat -",         &lv_font_montserrat_14, COL_MUTED, 420, 13);
    s_cntLbl    = mkLabel(scr, "TO 0",          &lv_font_montserrat_14, COL_MUTED, 590, 13);

    // ---- value tiles ----
    const char* capt[3] = { "POSITION", "CURRENT (raw)", "CMD POS" };
    lv_obj_t** vals[3] = { &s_posVal, &s_curVal, &s_cmdVal };
    lv_obj_t** subs[3] = { &s_posSub, &s_curSub, &s_cmdSub };
    for (int i = 0; i < 3; i++) {
        lv_obj_t* p = mkPanel(scr, 8 + i * 154, 40, 148, 92);
        mkLabel(p, capt[i], &lv_font_montserrat_14, COL_MUTED, 2, 0);
        *vals[i] = mkLabel(p, "-", &lv_font_montserrat_28, COL_TEXT, 2, 22);
        *subs[i] = mkLabel(p, "",  &lv_font_montserrat_14, COL_MUTED, 2, 58);
    }
    s_cfgLbl = mkLabel(scr, "PID -/-/-    LIM -  TRQ -", &lv_font_montserrat_14, COL_MUTED, 12, 140);

    // ---- slider block ----
    lv_obj_t* sp = mkPanel(scr, 8, 162, 452, 130);
    mkLabel(sp, "TARGET", &lv_font_montserrat_14, COL_MUTED, 2, 0);
    s_targetLbl = mkLabel(sp, "0", &lv_font_montserrat_28, COL_ACCENT, 80, 0);

    s_slider = lv_slider_create(sp);
    lv_obj_set_pos(s_slider, 8, 48);
    lv_obj_set_size(s_slider, 420, 22);
    lv_slider_set_range(s_slider, 0, asmg::kPositionMax);
    lv_slider_set_value(s_slider, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_slider, COL_ACCENT, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(s_slider, COL_TEXT, LV_PART_KNOB);
    // widen the touch target — gloved fingers on a resistive-feel panel
    lv_obj_set_ext_click_area(s_slider, 20);
    lv_obj_add_event_cb(s_slider, onSlider, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_add_event_cb(s_slider, onSlider, LV_EVENT_RELEASED, nullptr);

    mkLabel(sp, "live", &lv_font_montserrat_14, COL_MUTED, 8, 92);
    s_liveSw = mkSwitch(sp, 40, 86, [](lv_event_t*) {});
    s_speedDd = lv_dropdown_create(sp);
    lv_dropdown_set_options(s_speedDd, "speed 0000 (max)\nspeed 0100\nspeed 0300\nspeed 0500 (min)");
    lv_obj_set_pos(s_speedDd, 240, 82);
    lv_obj_set_size(s_speedDd, 190, 36);

    // ---- nudge + limits ----
    static const int32_t nudges[4] = { -1000, -100, 100, 1000 };
    static const char* nudgeTxt[4] = { "-1000", "-100", "+100", "+1000" };
    for (int i = 0; i < 4; i++)
        mkBtn(scr, nudgeTxt[i], 8 + i * 116, 300, 108, 40, onNudge, (void*)(intptr_t)nudges[i]);

    lv_obj_t* lp = mkPanel(scr, 8, 348, 452, 84);
    mkLabel(lp, "TRAVEL LIMITS (captured from reported pos; clamp the slider)",
            &lv_font_montserrat_14, COL_MUTED, 2, 0);
    s_limMinLbl = mkLabel(lp, "MIN ~0",     &lv_font_montserrat_14, COL_TEXT, 2, 30);
    s_limMaxLbl = mkLabel(lp, "MAX ~32767", &lv_font_montserrat_14, COL_TEXT, 2, 52);
    mkBtn(lp, "SET MIN", 150, 24, 90, 44, onSetMin, nullptr);
    mkBtn(lp, "SET MAX", 248, 24, 90, 44, onSetMax, nullptr);
    mkBtn(lp, "CLEAR",   346, 24, 90, 44, onClearLimits, nullptr);

    mkLabel(scr, "0x08/09/FC/FE not sendable from this tool. Slider release sends 0x01.",
            &lv_font_montserrat_14, COL_MUTED, 12, 444);

    // ---- right column: controls ----
    lv_obj_t* cp = mkPanel(scr, 468, 40, 324, 148);

    mkLabel(cp, "poll", &lv_font_montserrat_14, COL_MUTED, 2, 6);
    lv_obj_t* pollSw = mkSwitch(cp, 36, 0, onPollSw);
    (void)pollSw;
    lv_obj_t* rateDd = lv_dropdown_create(cp);
    lv_dropdown_set_options(rateDd, "10 ms\n20 ms\n50 ms\n100 ms\n200 ms");
    lv_dropdown_set_selected(rateDd, 1);
    lv_obj_set_pos(rateDd, 100, 0);
    lv_obj_set_size(rateDd, 96, 32);
    lv_obj_add_event_cb(rateDd, onRateDd, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_t* cmdDd = lv_dropdown_create(cp);
    lv_dropdown_set_options(cmdDd, "0x07 pos+cur\n0x02 pos+cmd");
    lv_obj_set_pos(cmdDd, 202, 0);
    lv_obj_set_size(cmdDd, 110, 32);
    lv_obj_add_event_cb(cmdDd, onCmdDd, LV_EVENT_VALUE_CHANGED, nullptr);

    mkLabel(cp, "addr", &lv_font_montserrat_14, COL_MUTED, 2, 46);
    mkBtn(cp, "-", 36, 38, 34, 32, onAddrDelta, (void*)(intptr_t)-1);
    s_addrLbl = mkLabel(cp, "0x00", &lv_font_montserrat_20, COL_TEXT, 78, 42);
    mkBtn(cp, "+", 130, 38, 34, 32, onAddrDelta, (void*)(intptr_t)+1);
    mkBtn(cp, "ADOPT --", 172, 38, 92, 32, onAdoptAddr, nullptr, &s_adoptBtnLbl);
    mkBtn(cp, "CLR", 272, 38, 40, 32, onClrCnt, nullptr);

    mkBtn(cp, "READ ID",  2,   78, 84, 32, onReadId, nullptr);
    mkBtn(cp, "READ PID", 94,  78, 84, 32, onReadPid, nullptr);
    mkBtn(cp, "READ CFG", 186, 78, 84, 32, onReadCfg, nullptr);

    mkLabel(cp, "LE", &lv_font_montserrat_14, COL_MUTED, 2, 118);
    lv_obj_t* endianSw = mkSwitch(cp, 24, 112, onEndianSw);
    (void)endianSw;
    mkLabel(cp, "USB log", &lv_font_montserrat_14, COL_MUTED, 96, 118);
    lv_obj_t* serialSw = mkSwitch(cp, 152, 112, onSerialSw);
    (void)serialSw;

    // ---- right column: trace ----
    lv_obj_t* tp = mkPanel(scr, 468, 194, 324, 278);
    mkLabel(tp, "BUS TRACE  (ms  </>/X  id  bytes)", &lv_font_montserrat_14, COL_MUTED, 2, 0);
    mkBtn(tp, "PAUSE", 240, 0, 70, 28, onPause, nullptr, &s_pauseBtnLbl);
    s_traceLbl = mkLabel(tp, "", &lv_font_montserrat_14, COL_TEXT, 2, 30);
    lv_obj_set_width(s_traceLbl, 306);
    lv_label_set_long_mode(s_traceLbl, LV_LABEL_LONG_CLIP);

    lv_timer_create(refreshTick, 100, nullptr);
}
