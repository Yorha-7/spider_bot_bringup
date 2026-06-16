# Changes Report

## Files Modified

### `include/big_bertha_hardware_bringup/mini_msgpack.hpp` (+106 lines)

**Packer — full msgpack format support** (previously threw "not implemented" for non-trivial values):
- `pack_int()`: handles uint8/16/32 (0xCC–0xCF), negative fixint (0xE0–0xFF), int8/16/32 (0xD0–0xD2), and int64 (0xD3)
- `pack_str()`: str8 (0xD9, strings up to 255 bytes), str16 (0xDA), str32 (0xDB) — previously limited to fixstr (31 chars)
- `pack_array()`: array16 (0xDC), array32 (0xDD) — previously limited to fixarray (15 elements)

**Unpacker — decode full msgpack wire format**:
- fixint negative (0xE0–0xFF) — was missing entirely, causes parse errors on firmware response IDs
- uint8/16/32 (0xCC–0xCF)
- int8/16/32/64 (0xD0–0xD3)
- str8/16/32 (0xD9–0xDB) — firmware uses str8 (0xD9) for IMU data

### `src/router_bridge.cpp` (+141/−50 lines)

**`recv_raw()` — rewritten from SO_RCVTIMEO to poll-based**:
- Before: `SO_RCVTIMEO` (1s) + `::read()` loop with intermediate msgpack parsing between chunks. First read often got only 4 bytes, parsing threw, subsequent read blocked for the full timeout → **0.8 Hz**.
- After: `poll()` with 5s timeout for first byte; after each read, immediately attempt parse — if complete, return with **zero extra latency**; if incomplete (firmware splits response into 2–3 chunks), `poll()` for just **10ms** for more data. → **54.5 Hz**.

**`call()` — persistent socket + robust error handling**:
- Socket is now **reused** across RPC calls (reconnect only on `sock_ < 0` after a failure)
- `next_id_` reset on connect (was missing — IDs could overflow/conflict)
- Error field parsing: handles `[code, message]` array format for firmware error responses
- Parse exceptions include raw hex dump for debugging

## Result

IMU streaming: **0.8 Hz → 54.5 Hz** (target: 30 Hz), zero errors or warnings.
