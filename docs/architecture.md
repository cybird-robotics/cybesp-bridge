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
learns the ground station's IP from the ground station's own traffic; see
[Ground station discovery](#ground-station-discovery).

---

## 2. Data flow

| Direction | Path | Framing | Port |
|-----------|------|---------|------|
| **Downlink** (telemetry) | UART0 RX → COBS-decode → queue → UDP → GS | COBS in, batch envelope out (one datagram per UART burst) | → GS :4210 |
| **Uplink** (commands) | UDP → COBS-encode → UART0 TX | raw in, COBS out | bind :4211 |
| **RTCM uplink** (RTK) — **highest priority** | UDP → pipe → UART1 TX (verbatim) | none (receiver parses RTCM3) | bind :4212 |
| **Time sync (SNTP)** | UDP NTP query → GS NTP server | NTP wire format | → GS :123 |
| **Time sync (PTP follow-up)** | UART0 TX every 3 s | COBS, msg_id 129 | — |
| **Datalink status** (bridge-originated) | UART0 TX at 5 Hz, from boot | COBS, msg_id 134 (`WireDatalinkStatus`: kind, WiFi state, AP RSSI, reconnects, uptime) | — |

Key constants live at the top of `src/bin/main.rs`: `DOWNLINK_PORT` (4210),
`UPLINK_PORT` (4211), `RTCM_UPLINK_PORT` (4212). WiFi credentials, the baud
rates and the antenna are settings; see
[Settings and USB shell](#settings-and-usb-shell).

### Ground station discovery and uplink lock

The ground station's IP is not configured, so any PC running `cybgcs` works
without reflashing. The first sender of a datagram to :4211 or :4212 becomes
the ground station (`accept_uplink`, stored in `GS_IP`), and telemetry (:4210)
and SNTP queries (:123) go there. `cybgcs` sends every drone a 10 Hz
fleet-state datagram from startup, so the bridge finds it within about 100 ms;
until then the downlink discards telemetry and SNTP waits.

The bridge then belongs to that ground station. Datagrams to :4211 or :4212
from any other sender are dropped before the flight controller sees them, and
counted (`uplink_foreign_drops`, shown by the shell's `stats`). The firmware ignores
sequence numbers, so it can't tell two ground stations' fleet states, Vicon
poses, origins or RTCM streams apart; without the lock, a second `cybgcs`
would interleave its own with the first one's. The rejected sender gets a busy
notice on its :4210 (`[0x00][0x80][owner IPv4]`, see
`cybflight_msgs::udp_batch`), at most once a second, which `cybgcs` shows as
`taken`. Another sender takes over only after the owner has been silent for
3 s (`GS_HANDOVER`).

Other PCs watch through the ground station instead (`cybgcs tui --observe`,
see the README): it forwards the drones' datagrams over the LAN, so the
drones' radio traffic doesn't grow with the number of viewers.

---

## 3. Software architecture

Built on **esp-hal 1.0** + **esp-rtos** (Embassy async runtime) for the
ESP32-C6 (RISC-V `riscv32imac-unknown-none-elf`). `main()` initializes
peripherals then spawns independent Embassy tasks; the main task idles.

### Boot sequence (`main`, `src/bin/main.rs`)

1. ESP-HAL init at max CPU clock.
2. Logger init (`esp-println`, USB-Serial-JTAG **only** — see below).
3. Heap allocator (96 KB — the WiFi stack, sized for its enlarged RX queue).
4. `board::init`: the board first hands over the flash peripheral, from which
   the settings are loaded (`settings::Store::open`); then it claims its UARTs
   and pins at the configured baud rates (STM32 link RX FIFO threshold 32;
   GNSS UART TX-only), sets up its status LED and antenna, and returns the
   rest. The UART halves stay Blocking here.
5. Start the esp-rtos scheduler (TIMG0 + software interrupt); spawn `led_task`.
6. Start the interrupt executors: SWI1 → RTCM writer (only if the board has a
   GNSS UART), SWI2 → STM32 link drain.
7–9. Init WiFi radio (power save off, RX/TX queues 12/6), create STA
   interface, configure `embassy-net` with DHCP.
10. Spawn the thread-mode tasks.

### Priorities

RTK corrections are the most important traffic on the link: a dropped or late
correction degrades the RTK solution, while lost telemetry only costs a
dashboard update. The scheduling follows from that, highest priority first:

| Level | Executor | Tasks | Why |
|-------|----------|-------|-----|
| 1 | interrupt, SWI1, `Priority3` | `rtcm_uart_writer_task` | Keeps UART1 saturated; nothing else can delay a correction on its way out. |
| 2 | interrupt, SWI2, `Priority2` | `fc_rx_drain_task` | Drains the STM32 link's 128 B UART FIFO before it overruns (≥1 ms of headroom at threshold 32). |
| 3 | thread mode | all network tasks | `embassy-net`'s `Stack` is `!Send`, so socket tasks cannot run on an interrupt executor. |

The interrupt-executor tasks only move bytes between a UART and a static
queue: no network access, no logging, no sleeps. The async UART driver types
are `!Send`, so each executor receives its UART half in Blocking mode through a
small bootstrap task that converts it with `into_async()` in place.

### Tasks (all `#[embassy_executor::task]`)

| Task | Executor | Role |
|------|----------|------|
| `rtcm_uart_writer_task` | SWI1 | `RTCM_PIPE` → UART1 TX. |
| `fc_rx_drain_task` | SWI2 | STM32 link RX → `FrameAccumulator` → whole decoded frames into `DL_QUEUE` (drops the oldest when full); marks each burst's end on RX idle. |
| `rtcm_uplink_task` | thread | UDP :4212 → whole datagram into `RTCM_PIPE`, or drop and count. Does nothing else, so the socket drains even while UART1 shifts out a 1 KB frame (~89 ms). |
| `downlink_task` | thread | `DL_QUEUE` → one batch datagram per UART burst → GS :4210. |
| `uplink_task` | thread | UDP :4211 → COBS encode → UART0 TX. Also emits PTP time-sync frames every 3 s, and the 5 Hz datalink status from boot (before the network is up — a bridge that never associates must still show in the FC's log). |
| `wifi_task` | thread | STA connect + auto-reconnect; drives `BRIDGE_STATUS`. While connected, polls `is_connected()` + the AP RSSI every 200 ms into `LINK_RSSI`; counts disconnects in `WIFI_RECONNECTS`. |
| `net_task` | thread | Runs the `embassy-net` stack. |
| `sntp_task` | thread | Polls the GS NTP server every 3 s; stores sync point in `NTP_STATE`. |
| `stats_task` | thread | Logs the `STATS` counters every 5 s, only after `stats on` in the shell (off at boot). |
| `shell_task` | thread | USB settings shell (`param …`, `reboot`); see [Settings and USB shell](#settings-and-usb-shell). |
| `led_task` | thread | Passes `BRIDGE_STATUS` to the board's `StatusLed` every 50 ms; the board picks the pattern. |

### Buffering and loss policy

| Buffer | Size | When full |
|--------|------|-----------|
| WiFi driver RX / TX queues | 12 / 6 packets | driver drops silently (sits before smoltcp) |
| RTCM socket RX | 16 datagrams / 8 KB | smoltcp drops silently |
| `RTCM_PIPE` → UART1 | 8 KB (~710 ms of line time) | whole datagram dropped, `rtcm_drops` counted |
| `DL_QUEUE` → WiFi | 64 decoded frames (~250 ms) | oldest frame dropped, counted |
| Downlink socket TX | 8 datagrams / 4 KB | `send_to` waits; only the downlink task stalls |

Every drop is whole-frame or whole-datagram, never a torn one: the telemetry
frames carry no CRC, and a spliced frame could decode as valid.

### Shared state

- `BRIDGE_STATUS: AtomicU8` — `BOOTING / WIFI_CONNECTING / WIFI_CONNECTED /
  WIFI_DISCONNECTED`. Set by `wifi_task`, read by `led_task` and sent to the
  FC as `WireDatalinkStatus::state` (values pinned by a const assert).
- `LINK_RSSI: AtomicI8` / `WIFI_RECONNECTS: AtomicU32` — AP RSSI [dBm]
  (`RSSI_UNKNOWN` when not associated) and disconnect count. Written by
  `wifi_task`, read by `uplink_task` for the datalink status frame and by
  the shell's `stats`.
- `NTP_STATE: Mutex<RefCell<Option<(i64, i64)>>>` — `(ntp_us, capture_instant_us)`.
  Written by `sntp_task`, read by `uplink_task` via `get_ntp_time_us()` which
  extrapolates from the monotonic clock between SNTP queries.
- `DL_QUEUE` (`embassy_sync::Channel`) and `RTCM_PIPE` (`embassy_sync::Pipe`,
  exactly one writer) — the hand-offs between the interrupt executors and the
  network tasks.
- `STATS: BridgeStats` — cumulative `AtomicU32` counters: UART0 errors, COBS
  errors, queue drops and high-water marks, datagrams sent, RTCM received /
  dropped / bytes to UART1. Shown by the shell's `stats` (and logged every 5 s
  after `stats on`); four of them ride in every
  downlink batch header, so `cybgcs` shows them per drone without a USB log.

### Wire protocol (COBS)

The UART0 leg uses **COBS** framing (0x00 delimiter), matching the cybflight
STM32 firmware. Raw frame layout: `[msg_id: u8][seq: u8][payload ≤ 128 B]`
(`MAX_RAW_FRAME = 130`). `FrameAccumulator::feed()` decodes incoming bytes
in-place on each 0x00; after a UART error or an oversized frame it discards
through the next delimiter rather than decode a partial frame. The RTCM leg is
**unframed** — bytes go straight to the receiver, which validates RTCM3 itself
(length + CRC24Q).

### Downlink batch envelope

Each UART burst from the STM32 (one per 10 ms tick) leaves as **one** UDP
datagram, defined in `cybflight_msgs::udp_batch`:

```
[0x00][version=1][uart_errors u16][dl_drops u16][rtcm_rx u16][rtcm_drops u16]
([len u8][frame: msg_id, seq, payload])*
```

msg_id 0 is reserved, so the first byte tells a batch from a legacy
single-frame datagram; `cybgcs` accepts both. A batch is flushed when the
UART goes idle on a frame boundary, after 1.5 ms without a frame, after 8 ms,
or at 1400 B. Set `DOWNLINK_BATCHING = false` to send one frame per datagram
for a ground station that predates the envelope — such a GCS discards batches.

### Logging

`esp-println` is built with `jtag-serial` only. Its default `auto` printer
falls back to UART0 whenever no USB host has been attached, and UART0 is the
STM32 link: every log line would be injected into the flight controller's
uplink. The default `critical-section` feature is also off, since it masked
interrupts for each print and would stall the high-priority UART tasks.

Log records are printed by `cybesp_bridge::logger` (`LEVEL - message`, the
level from `ESP_LOG` at build time) with CR LF line ends, so terminals such as
minicom show them properly; esp-println's own logger ends lines with a bare LF.

### Settings and USB shell

The WiFi SSID and password, `fc_baud`, `gnss_baud` and `ext_antenna` are
settings (`src/settings.rs`) with built-in defaults. They are stored in the
`nvs` flash partition, found through the partition table, as a
`sequential-storage` map keyed by a fixed id per setting, each value as the
text `param set` takes. At boot a missing or invalid stored value falls back
to its default. The board needs the bauds and the antenna while it claims its
pins, so `board::init` hands the flash peripheral to the application first.

`shell_task` reads lines from the USB-Serial-JTAG port (esp-hal driver,
receive half only) and prints through `esp_println`, like the logs. The
driver's transmit half is never used: it would interleave with esp-println's
direct FIFO writes. Commands: `param list | get | set | save | reset`,
`reboot`. Settings apply at the next boot.

Every flash write or erase runs with interrupts masked on this single-core
chip, so `param save` stalls the UART drains, the RTCM writer and WiFi while
it runs: expect some lost telemetry frames (counted as UART0 errors) and a
gap in corrections. The store is written only by that command.

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
| Antenna select | GPIO14 | — | LOW = built-in PCB antenna (default); HIGH = external u.FL with the `ext_antenna` setting — must match the fitted hardware, see [board_info.md](board_info.md#xiao-antenna-selection) |

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

Everything board-specific lives in `src/board/`, one file per board. The board
feature picks the file in `src/board/mod.rs`, the only board `cfg` in the
crate, and `build.rs` fails the build unless exactly one `board-*` feature is
enabled.

Each board file provides the same items:

- `NAME`, logged at boot.
- `HAS_ANTENNA_SWITCH`: whether the `ext_antenna` setting applies.
- `init(Peripherals, configure) -> Board`: first passes the flash peripheral
  to `configure`, which loads the settings and returns the `BoardConfig`;
  then claims the board's UARTs and pins, sets up its LED and antenna, and
  returns a `Board` holding the STM32 link halves (`fc_rx`, `fc_tx`), the
  GNSS TX half (`gnss_tx`, `None` on a board with no UART to spare), the
  `StatusLed`, and the peripherals the application still needs (WiFi, TIMG0,
  software interrupts, the USB port).
- `StatusLed::show(LinkStatus, tick)`, called by the single `led_task` every
  `board::LED_TICK_MS` (50 ms).

The **board** decides which UART instance and pins carry each link. The
**application** decides the link settings (`BoardConfig`: baud rates, FIFO
threshold) and never names a GPIO or UART instance. esp-hal 1.0's UART driver
types don't depend on the instance or the pins, so every board returns the
same types and the tasks don't change per board.

What a board lacks is a constant the settings check: with
`HAS_ANTENNA_SWITCH = false` (the Waveshare), `ext_antenna` is refused.

The RGB-only deps (`esp-hal-smartled`, `smart-leds`, `rgb`) are **optional** and
pulled in only by `board-waveshare`, so the XIAO build doesn't compile them.

To add a board, see [board_info.md](board_info.md#adding-a-board).

---

## 5. Building & flashing

Prerequisites: Rust stable with the `riscv32imac-unknown-none-elf` target and
`espflash` (`cargo install espflash`).

```sh
# Waveshare ESP32-C6-Zero (default)
cargo run --release

# Seeed XIAO ESP32-C6
cargo run --release --no-default-features --features board-xiao-c6```

On the XIAO, if it boots into download mode, power-cycle and run
`espflash monitor` separately (see the XIAO note above). Log verbosity is set
via the `ESP_LOG` env var (default `info`).

**Liveness check (board-independent):** watch the status LED. Any blink/solid
activity means the application is running — the ROM bootloader never drives it.
On the XIAO, **solid on = app running + WiFi connected**.
