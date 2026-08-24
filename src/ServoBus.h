// =============================================================================
//  ServoBus — TWAI transport + poll engine for the ASMG-MD servo sniffer
// =============================================================================
//  Bench/diagnostic use only. Bus: this display node + the servo, nothing else.
//  250 kbit/s (servo factory default), acceptance filter fully open so the
//  reply arbitration ID can be recorded (bring-up step 1 in the brief).
//
//  Threading: servoBusUpdate() runs on the protocol task (core 0). The UI task
//  (core 1) reads snapshots and queues requests through the setters below; all
//  shared state is guarded by a spinlock inside ServoBus.cpp.
//
//  Deliberately NOT reachable from this tool: 0x08 SaveCenter, 0x09 SetBitrate,
//  0xFC FactoryReset, 0xFE SetId — the destructive/commissioning writes.
//  0xFD ReadId is included: read-only and safe on a single-servo bus.
// =============================================================================
#pragma once

#include <Arduino.h>
#include "asmg_md_can.h"

#define SERVO_CAN_TX_PIN 17
#define SERVO_CAN_RX_PIN 18

// One raw frame as seen on (or offered to) the bus.
struct TraceEntry {
    uint32_t seq;
    uint32_t ms;        // millis() at capture
    uint32_t id;        // arbitration ID
    uint8_t  dlc;
    uint8_t  data[8];
    uint8_t  dir;       // TRACE_RX / TRACE_TX / TRACE_TX_FAIL
};
enum : uint8_t { TRACE_RX = 0, TRACE_TX = 1, TRACE_TX_FAIL = 2 };

struct ServoSnapshot {
    // Link
    bool     replyFresh;      // valid reply within the last 500 ms
    uint32_t lastAnyRxId;     // arbitration ID of the last RX frame — record this!
    uint32_t lastGoodMs;      // millis() of last parsed reply, 0 = never
    uint32_t timeouts;        // total reply timeouts
    uint32_t txFails;         // twai_transmit() refusals (queue full / bus dead).
                              // NOTE: single-shot TX still "succeeds" without an
                              // ACK — a missing servo shows up as timeouts, not
                              // txFails. Check busErrTx for ACK errors.
    uint32_t rxCount;
    uint32_t txCount;
    // Request→reply latency, microseconds (quantised by the 2 ms task period)
    uint32_t latLastUs;
    uint32_t latMaxUs;
    // Decoded values (using the runtime endian setting)
    uint16_t position;        // 0x02 / 0x07 current position
    uint16_t cmdPosition;     // 0x02 commanded position
    uint16_t current;         // 0x07 present current (units unknown — raw)
    uint16_t torque;          // 0x04 present torque (raw)
    uint16_t currentLimit;    // 0x04 configured limit (raw)
    uint16_t pidP, pidI, pidD;
    uint8_t  discoveredAddr;  // 0xFD reply byte 0 (provisional decode)
    uint8_t  replyEchoCmd04;  // what cmd byte the 0x04 reply actually echoed (0=none seen)
    bool     havePos, haveCmdPos, haveCur, haveCfg, havePid, haveAddr;
    // TWAI health
    uint8_t  busState;        // twai_state_t
    uint32_t busErrTx, busErrRx;
    uint32_t busRecoveries;
};

void servoBusBegin();
void servoBusUpdate();                    // call every ~2 ms from the protocol task
ServoSnapshot servoGetSnapshot();

// Trace: UI passes the last seq it has seen; new entries are copied out.
// Returns the number written; *lastSeq is advanced.
uint32_t servoTraceRead(uint32_t* lastSeq, TraceEntry* out, uint32_t maxOut);

// --- UI-side controls (thread-safe) ---
void servoSetAddress(uint8_t addr);
uint8_t servoGetAddress();
void servoSetBigEndian(bool be);          // runtime word-order toggle (bring-up step 3)
bool servoGetBigEndian();
void servoSetPolling(bool on);
bool servoGetPolling();
void servoSetPollCmd(uint8_t cmd);        // 0x07 (pos+current) or 0x02 (pos+cmdpos)
void servoSetPollIntervalMs(uint16_t ms);
void servoSetSerialMirror(bool on);       // print every frame on USB serial
bool servoGetSerialMirror();

// One-shot actions (queued, sent when the bus is idle; one outstanding request)
void servoCommandPosition(uint16_t counts, uint16_t speed);   // 0x01, no reply
void servoRequestReadId();                                    // 0xFD broadcast
void servoRequestReadPid();                                   // 0x06
void servoRequestReadCfg();                                   // 0x04
void servoClearCounters();                                    // timeouts/txfails/latency
