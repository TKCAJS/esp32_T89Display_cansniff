// =============================================================================
//  asmg_md_can.h  —  Wingxine ASMG-MD (ZDY native CAN) servo protocol codec
// =============================================================================
//  T89 motorsport electronics — clutch actuator interface
//
//  PURE CODEC. No hardware dependencies, no Arduino headers, no dynamic
//  allocation, no blocking. Every function is inline and side-effect free so
//  this file can be unit-tested on the host as well as compiled for STM32/ESP32.
//
//  Transport (FDCAN / TWAI) lives elsewhere. See ASMG_MD_INTEGRATION_BRIEF.md.
//
//  PROTOCOL SOURCE: manufacturer PDF "CAN协议" (ASMG-MD).
//  The document is terse and leaves several things unstated. Everything marked
//  [VERIFY] below MUST be confirmed on the bench before this is trusted with a
//  clutch. Do not assume; sniff the bus.
// =============================================================================

#pragma once

#include <stdint.h>

// -----------------------------------------------------------------------------
//  Endianness of 16-bit parameters inside the payload.
//
//  The manufacturer doc writes multi-byte values as "xx xx" without stating
//  byte order. Big-endian (MSB first) is assumed because the documented
//  position ceiling is 0x7FFF and the speed floor is 0x0500, both of which read
//  naturally MSB-first. THIS IS AN ASSUMPTION. [VERIFY]
//
//  Bench test: command position 0x0100 (256). If the servo travels to roughly
//  1/128 of full scale, big-endian is correct. If it slams toward ~half scale,
//  the device is little-endian — set ASMG_WORD_BIG_ENDIAN to 0 and retest.
//  Do this with the output arm DISCONNECTED from the clutch.
// -----------------------------------------------------------------------------
#ifndef ASMG_WORD_BIG_ENDIAN
#define ASMG_WORD_BIG_ENDIAN 1
#endif

namespace asmg {

// -----------------------------------------------------------------------------
//  Bus constants
// -----------------------------------------------------------------------------

/// Arbitration ID used by the host for ALL commands. 29-bit extended, J1939
/// PDU2-shaped. Per-servo addressing is done in payload byte 0, NOT in the ID.
constexpr uint32_t kHostCanId = 0x18EF0201UL;

/// Arbitration ID the servo replies on. [VERIFY] — the manufacturer doc does
/// not state this. It may echo kHostCanId, or it may swap the source/dest
/// bytes per J1939 convention (0x18EF0102). Bring-up step 1 is to sniff with a
/// fully open acceptance filter and record the real value, then fix this here.
constexpr uint32_t kReplyCanIdAssumed = 0x18EF0201UL;

/// Every documented frame is 8 bytes; unused trailing bytes are pad-zero.
constexpr uint8_t kDlc = 8;

/// Factory default bit rate. 250 kbit/s.
constexpr uint32_t kDefaultBitrate = 250000UL;

// -----------------------------------------------------------------------------
//  Addressing
// -----------------------------------------------------------------------------

/// Broadcast address, written to payload byte 0. Causes ALL servos on the bus
/// to act simultaneously.
///
/// DANGER: only ever use this for commands with NO reply (0x01 set position).
/// If two or more servos answer a broadcast, they transmit different payloads
/// under the SAME arbitration ID at the same instant, which is a bit-level
/// collision that classic CAN cannot arbitrate — the result is error frames,
/// not two messages. See the brief.
constexpr uint8_t kAddrBroadcast = 0xFE;

/// Address 0x00 is used throughout the manufacturer's own examples, so it is
/// presumed to be the factory default. [VERIFY] with cmdReadId().
constexpr uint8_t kAddrFactoryDefault = 0x00;

// -----------------------------------------------------------------------------
//  Command bytes (payload byte 1)
// -----------------------------------------------------------------------------

enum class Cmd : uint8_t {
    SetPositionSpeed  = 0x01,  ///< write pos+speed, no reply, broadcast-capable
    ReadPosition      = 0x02,  ///< -> current position + commanded position
    SetCurrent        = 0x03,  ///< write current limit, no reply
    ReadCurrentConfig = 0x04,  ///< -> present torque + configured current
    SetPid            = 0x05,  ///< write P/I/D, no reply
    ReadPid           = 0x06,  ///< -> P/I/D
    ReadPositionAndI  = 0x07,  ///< -> current position + present current
    SaveCenter        = 0x08,  ///< persist centre position, no reply
    SetBitrate        = 0x09,  ///< change CAN bit rate, no reply
    FactoryReset      = 0xFC,  ///< broadcast-only per doc
    ReadId            = 0xFD,  ///< broadcast-only per doc
    SetId             = 0xFE,  ///< NOTE: byte 0 carries the NEW address
};

/// Bit-rate selector for Cmd::SetBitrate.
enum class Bitrate : uint8_t {
    k250k = 0x00,
    k500k = 0x01,
    k1M   = 0x02,
};

// -----------------------------------------------------------------------------
//  Parameter ranges (from the manufacturer doc)
// -----------------------------------------------------------------------------

constexpr uint16_t kPositionMin   = 0x0000;
constexpr uint16_t kPositionMax   = 0x7FFF;  ///< 15-bit, 32767 counts full scale

constexpr uint16_t kSpeedFastest  = 0x0000;  ///< 0 = fastest
constexpr uint16_t kSpeedSlowest  = 0x0500;  ///< 1280 = slowest

constexpr uint16_t kCenterRatioMin = 0x0000;
constexpr uint16_t kCenterRatioMax = 0x03E8;  ///< 1000

// Units for speed and current are NOT documented. Treat both as opaque
// device-specific scalars until characterised on the bench. [VERIFY]

// -----------------------------------------------------------------------------
//  Frame container — transport-agnostic
// -----------------------------------------------------------------------------

struct Frame {
    uint32_t id;        ///< always extended (29-bit)
    uint8_t  dlc;
    uint8_t  data[8];
};

// -----------------------------------------------------------------------------
//  Word packing helpers
// -----------------------------------------------------------------------------

inline void packWord(uint8_t* p, uint16_t v) {
#if ASMG_WORD_BIG_ENDIAN
    p[0] = static_cast<uint8_t>(v >> 8);
    p[1] = static_cast<uint8_t>(v & 0xFF);
#else
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>(v >> 8);
#endif
}

inline uint16_t unpackWord(const uint8_t* p) {
#if ASMG_WORD_BIG_ENDIAN
    return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
#else
    return static_cast<uint16_t>((static_cast<uint16_t>(p[1]) << 8) | p[0]);
#endif
}

inline uint16_t clampU16(uint16_t v, uint16_t lo, uint16_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// -----------------------------------------------------------------------------
//  Frame builders
// -----------------------------------------------------------------------------

/// Zero-filled frame with address and command populated.
inline Frame makeFrame(uint8_t addr, Cmd cmd) {
    Frame f{};
    f.id      = kHostCanId;
    f.dlc     = kDlc;
    f.data[0] = addr;
    f.data[1] = static_cast<uint8_t>(cmd);
    // data[2..7] already zero
    return f;
}

/// 0x01 — set target position and travel speed. No reply. Broadcast-capable.
/// Inputs are clamped to the documented ranges; callers should ALSO clamp to
/// their own calibrated mechanical limits before calling. See the brief.
inline Frame cmdSetPosition(uint8_t addr, uint16_t position, uint16_t speed) {
    Frame f = makeFrame(addr, Cmd::SetPositionSpeed);
    packWord(&f.data[2], clampU16(position, kPositionMin, kPositionMax));
    packWord(&f.data[4], clampU16(speed, kSpeedFastest, kSpeedSlowest));
    return f;
}

/// 0x02 — request current position and last commanded position.
inline Frame cmdReadPosition(uint8_t addr) {
    return makeFrame(addr, Cmd::ReadPosition);
}

/// 0x03 — set the current (torque) limit. No reply.
inline Frame cmdSetCurrent(uint8_t addr, uint16_t current) {
    Frame f = makeFrame(addr, Cmd::SetCurrent);
    packWord(&f.data[2], current);
    return f;
}

/// 0x04 — request present torque and configured current limit.
inline Frame cmdReadCurrentConfig(uint8_t addr) {
    return makeFrame(addr, Cmd::ReadCurrentConfig);
}

/// 0x05 — set PID gains. No reply.
inline Frame cmdSetPid(uint8_t addr, uint16_t p, uint16_t i, uint16_t d) {
    Frame f = makeFrame(addr, Cmd::SetPid);
    packWord(&f.data[2], p);
    packWord(&f.data[4], i);
    packWord(&f.data[6], d);
    return f;
}

/// 0x06 — request PID gains.
inline Frame cmdReadPid(uint8_t addr) {
    return makeFrame(addr, Cmd::ReadPid);
}

/// 0x07 — request current position and present current draw.
/// This is the primary telemetry poll for the clutch control loop.
inline Frame cmdReadPositionAndCurrent(uint8_t addr) {
    return makeFrame(addr, Cmd::ReadPositionAndI);
}

/// 0x08 — persist the centre position. No reply.
/// DANGER: writes non-volatile configuration. Commissioning use only, never
/// from the runtime control path. Gate behind service mode.
inline Frame cmdSaveCenter(uint8_t addr, uint16_t ratio) {
    Frame f = makeFrame(addr, Cmd::SaveCenter);
    packWord(&f.data[2], clampU16(ratio, kCenterRatioMin, kCenterRatioMax));
    return f;
}

/// 0x09 — change the servo's CAN bit rate. No reply.
/// DANGER: the servo drops off the bus at the old rate immediately after this.
/// Power-cycle and re-open the peripheral at the new rate. If you get this
/// wrong you will need a variable-rate sniffer to find the device again.
/// Commissioning use only, gated behind service mode.
inline Frame cmdSetBitrate(uint8_t addr, Bitrate rate) {
    Frame f = makeFrame(addr, Cmd::SetBitrate);
    f.data[2] = static_cast<uint8_t>(rate);
    return f;
}

/// 0xFC — factory reset.
/// DANGER: destroys stored ID, centre, PID and bit rate. Doc states this is a
/// broadcast command that must not be issued on a multi-device bus.
/// Commissioning use only, gated behind service mode.
inline Frame cmdFactoryReset(uint8_t addr = kAddrBroadcast) {
    return makeFrame(addr, Cmd::FactoryReset);
}

/// 0xFD — read the current device address. Sent to the broadcast address
/// because the caller does not yet know the real one.
/// Doc states this must not be issued on a multi-device bus — with more than
/// one servo attached, multiple simultaneous replies will collide.
inline Frame cmdReadId() {
    return makeFrame(kAddrBroadcast, Cmd::ReadId);
}

/// 0xFE — assign a new device address. Byte 0 carries the NEW address, which
/// is a different role from every other command in this protocol.
/// DANGER: writes non-volatile configuration, and per the doc must not be
/// issued on a multi-device bus. Commissioning use only.
inline Frame cmdSetId(uint8_t newAddr) {
    return makeFrame(newAddr, Cmd::SetId);
}

// -----------------------------------------------------------------------------
//  Reply parsers
//
//  All parsers are strict: they check the address byte matches the servo that
//  was polled, and reject anything else. They do NOT check the arbitration ID
//  — the transport layer is responsible for only feeding in frames that
//  actually came from the servo bus.
// -----------------------------------------------------------------------------

struct PositionReply {
    uint16_t currentPosition;
    uint16_t commandedPosition;
};

struct CurrentConfigReply {
    uint16_t presentTorque;
    uint16_t configuredCurrent;
};

struct PidReply {
    uint16_t p;
    uint16_t i;
    uint16_t d;
};

struct PositionCurrentReply {
    uint16_t currentPosition;
    uint16_t presentCurrent;
};

/// Parse a 0x02 reply.
inline bool parsePosition(const uint8_t* d, uint8_t expectedAddr,
                          PositionReply& out) {
    if (d[0] != expectedAddr) return false;
    if (d[1] != static_cast<uint8_t>(Cmd::ReadPosition)) return false;
    out.currentPosition   = unpackWord(&d[2]);
    out.commandedPosition = unpackWord(&d[4]);
    return true;
}

/// Parse a 0x04 reply.
///
/// NOTE: the manufacturer doc shows the REPLY to command 0x04 echoing command
/// byte 0x03, not 0x04. That is most likely a documentation typo, but it might
/// be real firmware behaviour — so both are accepted here. Once the bench
/// confirms which one the device actually sends, tighten this. [VERIFY]
inline bool parseCurrentConfig(const uint8_t* d, uint8_t expectedAddr,
                               CurrentConfigReply& out) {
    if (d[0] != expectedAddr) return false;
    const uint8_t c = d[1];
    if (c != static_cast<uint8_t>(Cmd::ReadCurrentConfig) &&
        c != static_cast<uint8_t>(Cmd::SetCurrent)) return false;
    out.presentTorque     = unpackWord(&d[2]);
    out.configuredCurrent = unpackWord(&d[4]);
    return true;
}

/// Parse a 0x06 reply.
inline bool parsePid(const uint8_t* d, uint8_t expectedAddr, PidReply& out) {
    if (d[0] != expectedAddr) return false;
    if (d[1] != static_cast<uint8_t>(Cmd::ReadPid)) return false;
    out.p = unpackWord(&d[2]);
    out.i = unpackWord(&d[4]);
    out.d = unpackWord(&d[6]);
    return true;
}

/// Parse a 0x07 reply. Primary telemetry path.
inline bool parsePositionAndCurrent(const uint8_t* d, uint8_t expectedAddr,
                                    PositionCurrentReply& out) {
    if (d[0] != expectedAddr) return false;
    if (d[1] != static_cast<uint8_t>(Cmd::ReadPositionAndI)) return false;
    out.currentPosition = unpackWord(&d[2]);
    out.presentCurrent  = unpackWord(&d[4]);
    return true;
}

/// Parse a 0xFD reply.
///
/// The doc shows the reply as "FE FD" and does not say where the discovered
/// address actually appears. It may be in byte 0 (in which case the doc's "FE"
/// is just placeholder notation), or in one of the trailing bytes, or nowhere
/// at all. Sniff the raw frame during bring-up and rewrite this. [VERIFY]
inline bool parseReadId(const uint8_t* d, uint8_t& discoveredAddr) {
    if (d[1] != static_cast<uint8_t>(Cmd::ReadId)) return false;
    discoveredAddr = d[0];   // provisional — confirm on the bench
    return true;
}

// -----------------------------------------------------------------------------
//  Reply classification helper
//
//  Because every reply shares one arbitration ID, the transport cannot use CAN
//  filters to tell responses apart. Dispatch on the command byte instead.
// -----------------------------------------------------------------------------

/// True if this command produces a reply frame at all. Commands that do not
/// reply must never be waited on — the driver will hang or false-fault.
inline bool commandExpectsReply(Cmd c) {
    switch (c) {
        case Cmd::ReadPosition:
        case Cmd::ReadCurrentConfig:
        case Cmd::ReadPid:
        case Cmd::ReadPositionAndI:
        case Cmd::FactoryReset:
        case Cmd::ReadId:
        case Cmd::SetId:
            return true;
        default:
            return false;   // 0x01, 0x03, 0x05, 0x08, 0x09
    }
}

/// True if this command writes non-volatile config or alters bus parameters,
/// and must therefore be gated behind service mode and never reachable from
/// the runtime control path.
inline bool commandIsDestructive(Cmd c) {
    switch (c) {
        case Cmd::SaveCenter:
        case Cmd::SetBitrate:
        case Cmd::FactoryReset:
        case Cmd::SetId:
            return true;
        default:
            return false;
    }
}

}  // namespace asmg
