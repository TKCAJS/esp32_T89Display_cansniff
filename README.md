# esp32_T89Display_cansniff

Temporary diagnostic firmware for the T89 display node (ESP32-S3 8048S043C):
a touchscreen CAN sniffer/exerciser for the **Wingxine ASMG-MD** servo (native
CAN "ZDY" variant). Its job is to resolve every **[VERIFY]** item in
`esp32_T89Display/docs/ASMG_MD_INTEGRATION_BRIEF.md` before the real driver is
written. When done, reflash the node with `esp32_T89Display`.

Bus setup: display node + servo only, 250 kbit/s (servo factory default),
acceptance filter fully open. CAN pins TX=17 RX=18 (same as the main firmware).
Bench only — **output arm disconnected**, current-limited supply.

## What it can and cannot send

Sendable: `0x01` set position+speed, `0x02`/`0x07` polls, `0x04` read config,
`0x06` read PID, `0xFD` read ID (read-only broadcast, safe on a one-servo bus).

**Not sendable, by design:** `0x03` set current, `0x05` set PID, `0x08` save
centre, `0x09` set bitrate, `0xFC` factory reset, `0xFE` set ID. Nothing in
this tool writes servo state beyond commanding motion.

## UI map

- **Status bar** — link state, **arbitration ID of the last received frame**
  (bring-up step 1 — record it), request→reply latency last/max (step 7),
  timeout / TX-fail / TWAI error counters.
- **Tiles** — reported position (counts/hex/%), current (raw), commanded
  position. PID and current-limit readbacks below, including which command
  byte the `0x04` reply actually echoed (step 8).
- **Slider + nudges** — sends `0x01` on release; `live` switch streams at
  10 Hz while dragging; speed dropdown 0x0000/0x0100/0x0300/0x0500 (step 5).
- **Travel limits** — SET MIN / SET MAX capture the current *reported*
  position and clamp the slider to that window (step 4). Display-side only;
  the servo has no limit registers. Write the numbers down — they become
  `travelMinCounts` / `travelMaxCounts` in `ClutchCalibration`.
- **Controls** — poll on/off (off at boot), rate, poll command, payload
  address with ADOPT from the `0xFD` reply (step 2), **LE switch** to flip
  16-bit word order at runtime (step 3), USB log mirror.
- **Bus trace** — last frames raw: ms, direction (`<` rx, `>` tx, `X` tx
  fail), arbitration ID, 8 payload bytes. PAUSE freezes the view.

## Bring-up sequence with this tool

1. Poll OFF, press READ ID → note the reply's arbitration ID in the status
   bar and the raw bytes in the trace → fixes `kReplyCanIdAssumed` and
   `parseReadId` in `lib/AsmgServo/asmg_md_can.h`.
2. ADOPT the discovered address (or leave 0x00).
3. Slider to 256 (0x0100), slowest speed, send. Small move → big-endian
   confirmed. Half-scale lunge → flip the LE switch and retest. Bake the
   result into `ASMG_WORD_BIG_ENDIAN`.
4. Enable polling (0x07, 20 ms). Nudge across the range, capture MIN/MAX at
   the mechanical stops.
5. Time full-travel moves at each speed value; watch latency max over a few
   thousand polls.
6. Load/stall current readings come from the CURRENT tile (raw units).
7. Record everything in `tools/asmg_commission.md` (per the brief, §8).

## Relationship to esp32_T89Display

Copied unchanged: board json, `lv_conf.h`, `copy_lv_conf.py` (bounce-buffer
panel patch), `sdkconfig.defaults`, touch calibration, task/core layout,
`lib/AsmgServo/asmg_md_can.h`. The dashboard UI, can_ids, SD logging and pit
server are deliberately absent. Do not merge changes back except corrections
to `asmg_md_can.h` once bench-verified.
