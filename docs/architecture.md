# cybesp-bridge — Architecture & Pinout

A message-agnostic **WiFi ↔ UART bridge** for the cybflight flight stack. It
forwards COBS-framed UART traffic between an STM32H743 flight controller and a
ground station over WiFi (raw UDP), relays RTCM3 GPS corrections to a ublox
receiver, and provides NTP-based time sync. It does **not** interpret message
payloads — it only frames, forwards, and time-stamps.

Runs on the **ESP32-C6** in `no_std` Rust. Two physical boards are supported and
selected at build time via a Cargo feature; see [Boards & Pinout](#boards--pinout).

---

## 1. System context

```
        STM32H743                ESP32-C6 (this firmware)            Ground station PC
   ┌──────────────────┐        ┌──────────────────────────┐       ┌────────────────────┐
   │ USART6           │◀──────▶│ UART0  (COBS framed)      │       │  cybgcs            │
   │  (telemetry/cmd) │  921600│                           │       │                    │
   └──────────────────┘        │   downlink ─ UDP :4210 ──▶│──────▶│ telemetry sink     │
                               │   uplink   ◀ UDP :4211 ──│◀──────│ command source     │
   ┌──────────────────┐        │   rtcm     ◀ UDP :4212 ──│◀──────│ NTRIP/RTCM source  │
   │ ublox GPS UART2  │◀───────│ UART1  (raw RTCM3)        │       │ NTP server :123    │
   │  (RTCM3 in)      │  115200│   sntp     ─ UDP :123  ──▶│◀─────▶│ (time sync)        │
   └──────────────────┘        │              WiFi STA / DHCP      │                    │
                               └──────────────────────────┘       └────────────────────┘
```

The bridge joins a WiFi network in **STA mode**, obtains an IP via **DHCP**, and
talks to a fixed ground-station IP (`GROUND_STATION_IP` in `src/bin/main.rs`).

---

## 2. Data flow

| Direction | Path | Framing | Port |
|-----------|------|---------|------|
| **Downlink** (telemetry) | UART0 RX → COBS-decode → UDP → GS | COBS in, raw payload out | → GS :4210 |
| **Uplink** (commands) | UDP → COBS-encode → UART0 TX | raw in, COBS out | bind :4211 |
| **RTCM uplink** (RTK) | UDP → UART1 TX (verbatim) | none (ublox parses RTCM3) | bind :4212 |
| **Time sync (SNTP)** | UDP NTP query → GS NTP server | NTP wire format | → GS :123 |
| **Time sync (PTP follow-up)** | UART0 TX every 3 s | COBS, msg_id 129 | — |

Key constants live at the top of `src/bin/main.rs`: `WIFI_SSID`,
`WIFI_PASSWORD`, `GROUND_STATION_IP`, `DOWNLINK_PORT` (4210), `UPLINK_PORT`
(4211), `RTCM_UPLINK_PORT` (4212).

---

## 3. Software architecture

Built on **esp-hal 1.0** + **esp-rtos** (Embassy async runtime) for the
ESP32-C6 (RISC-V `riscv32imac-unknown-none-elf`). `main()` initializes
peripherals then spawns independent Embassy tasks; the main task idles.

### Boot sequence (`main`, `src/bin/main.rs`)

1. ESP-HAL init at max CPU clock.
2. Logger init (`esp-println`, over USB-Serial-JTAG).
3. Heap allocator (72 KB — required by the WiFi stack).
4. Start the esp-rtos scheduler (TIMG0 + software interrupt).
5. Init status LED → spawn `led_task`.
6. Init UART0 (STM32, 921600) and UART1 (ublox RTCM, 115200, TX-only).
6b. *(XIAO only)* configure the antenna RF switch.
7–9. Init WiFi radio, create STA interface, configure `embassy-net` with DHCP.
10. Spawn all tasks.

### Tasks (all `#[embassy_executor::task]`)

| Task | Role |
|------|------|
| `wifi_task` | STA connect + auto-reconnect; drives `BRIDGE_STATUS`. |
| `net_task` | Runs the `embassy-net` stack. |
| `downlink_task` | UART0 RX → `FrameAccumulator` (COBS decode) → UDP to GS :4210. |
| `uplink_task` | UDP :4211 → COBS encode → UART0 TX. Also emits PTP time-sync frames every 3 s. |
| `rtcm_uplink_task` | UDP :4212 → UART1 TX verbatim (no framing). |
| `sntp_task` | Polls the GS NTP server every 3 s; stores sync point in `NTP_STATE`. |
| `led_task` | Renders `BRIDGE_STATUS` on the board LED (board-specific). |

### Shared state

- `BRIDGE_STATUS: AtomicU8` — `BOOTING / WIFI_CONNECTING / WIFI_CONNECTED /
  WIFI_DISCONNECTED`. Set by `wifi_task`, read by `led_task`.
- `NTP_STATE: Mutex<RefCell<Option<(i64, i64)>>>` — `(ntp_us, capture_instant_us)`.
  Written by `sntp_task`, read by `uplink_task` via `get_ntp_time_us()` which
  extrapolates from the monotonic clock between SNTP queries.

### Wire protocol (COBS)

The UART0 leg uses **COBS** framing (0x00 delimiter), matching the cybflight
STM32 firmware. Raw frame layout: `[msg_id: u8][seq: u8][payload ≤ 128 B]`
(`MAX_RAW_FRAME = 130`). `FrameAccumulator::feed()` decodes incoming bytes
in-place on each 0x00. The RTCM leg is **unframed** — bytes go straight to the
ublox, which validates RTCM3 itself (length + CRC24Q).

### Time sync (NTP + PTP follow-up)

`sntp_task` queries the GS NTP server (UDP :123) and records UTC-vs-monotonic
offset. `uplink_task` sends a `WireTimeSync` frame (`msg_id = 129`) to the STM32
every 3 s carrying the ESP send timestamp and the previous frame's
TX-complete timestamp (a PTP-style follow-up). The STM32 smooths the residual
DMA-vs-shift-register bias with an EMA.

---

## 4. Boards & Pinout

Both boards use the **ESP32-C6** MCU and native USB for flashing/logging. The
shared logic (UART0, WiFi, COBS, NTP) is identical; only the **RTCM pin**,
**status LED**, and **antenna handling** differ. The target is chosen by a
Cargo feature:

| Board | Feature | Default |
|-------|---------|---------|
| Waveshare ESP32-C6-Zero | `board-waveshare` | ✅ |
| Seeed XIAO ESP32-C6 | `board-xiao-c6` | |

A `compile_error!` guard enforces exactly one board feature.

### Waveshare ESP32-C6-Zero

| Function | GPIO | Notes |
|----------|------|-------|
| UART0 TX → STM32 USART6 RX (PC7) | GPIO16 | 921600 8N1, COBS, bidirectional |
| UART0 RX ← STM32 USART6 TX (PC6) | GPIO17 | |
| UART1 TX → ublox UART2 (RTCM3) | GPIO14 | 115200, TX-only, unframed |
| Status LED | GPIO8 | WS2812 **RGB** via RMT channel 0 |
| Antenna | — | onboard, no GPIO control |

LED patterns: breathing blue = booting/connecting, solid dim green = connected,
blinking red = disconnected. (The onboard WS2812 uses RGB wire order, so the
color fields are remapped — see `led_task`.)

### Seeed XIAO ESP32-C6

| Function | GPIO | XIAO pin | Notes |
|----------|------|----------|-------|
| UART0 TX → STM32 | GPIO16 | D6 | 921600 8N1, COBS, bidirectional |
| UART0 RX ← STM32 | GPIO17 | D7 | |
| UART1 TX → ublox (RTCM3) | GPIO0 | D0 | 115200, TX-only (GPIO14 is the antenna switch) |
| Status LED (user) | GPIO15 | — | single **monochrome** LED, **active-low**, blink-rate status |
| RF switch enable | GPIO3 | — | held LOW |
| Antenna select | GPIO14 | — | held LOW = built-in PCB antenna (HIGH = external u.FL) |

LED patterns (no color → status by blink rate): slow ~1 Hz blink =
booting/connecting, **solid on = connected**, fast ~5 Hz blink = disconnected.
LED polarity is in the single `LED_ON` / `LED_OFF` const.

**XIAO-specific notes**

- **GPIO15 is an ESP32-C6 strapping pin** (gates ROM boot-log printing). Seeed
  wires the user LED there deliberately; using it as an output post-boot is fine.
- **Onboard red LED** = the LiPo charge-status indicator (charger IC, *not* a
  GPIO). With no battery it lights ~10 s then turns off — unrelated to firmware.
- **Native-USB download-mode quirk:** `espflash`'s auto-reset after flashing
  can leave the chip in the ROM serial-download bootloader
  (`boot:0x16 (DOWNLOAD)`, "waiting for download"). To actually run the app,
  **power-cycle** (unplug/replug USB with BOOT not held), then attach a
  non-resetting monitor with `espflash monitor`. A "Broken pipe" on reset is
  just the USB-Serial-JTAG re-enumerating.

### How board differences are implemented

All divergence is `#[cfg(feature = ...)]`-gated in `src/bin/main.rs` at three
sites — there is no separate board-abstraction module:

1. **LED** — two cfg-gated `led_task` variants (WS2812 `SmartLedsAdapter` vs
   plain `Output`) and two cfg-gated init blocks in `main`. Embassy tasks don't
   play well with generics, so two concrete functions are used instead of a trait.
2. **RTCM UART1 TX pin** — `.with_tx(GPIO14)` (Waveshare) vs `.with_tx(GPIO0)` (XIAO).
3. **Antenna** — XIAO-only block driving GPIO3/GPIO14 before WiFi init.

The RGB-only deps (`esp-hal-smartled`, `smart-leds`, `rgb`) are **optional** and
pulled in only by `board-waveshare`, so the XIAO build doesn't compile them.

---

## 5. Building & flashing

Prerequisites: Rust stable with the `riscv32imac-unknown-none-elf` target and
`espflash` (`cargo install espflash`).

```sh
# Waveshare ESP32-C6-Zero (default)
cargo run --release

# Seeed XIAO ESP32-C6
cargo run --release --no-default-features --features board-xiao-c6
```

On the XIAO, if it boots into download mode, power-cycle and run
`espflash monitor` separately (see the XIAO note above). Log verbosity is set
via the `ESP_LOG` env var (default `info`).

**Liveness check (board-independent):** watch the status LED. Any blink/solid
activity means the application is running — the ROM bootloader never drives it.
On the XIAO, **solid on = app running + WiFi connected**.
