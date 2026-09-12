# cybesp-bridge

Message-agnostic WiFi-UART bridge for the cybflight flight controller.

Runs on ESP32-C6 boards (Waveshare ESP32-C6-Zero, Seeed XIAO ESP32-C6) and
forwards COBS-framed UART data from an STM32H743 to UDP over WiFi, and vice
versa. The bridge does
not interpret message contents — if new telemetry types are added to the flight
controller firmware, the bridge does not need reflashing.

## Data Flow

```
Ground Station (Python)               ESP32-C6                    STM32H743
───────────────────────               ────────                    ─────────
                                      WiFi STA
UDP RX :4210  ◄──── UDP datagram ◄─── COBS decode ◄── UART RX ◄── COBS TX
UDP TX :4211  ────► UDP datagram ───► COBS encode ──► UART TX ──► COBS RX
```

- **Downlink** (telemetry): STM32 sends COBS-encoded frames over UART at
  921600 baud, one burst per 10 ms tick. The ESP32 decodes COBS framing and
  sends each burst as one batch datagram to the ground station on port 4210.
- **RTCM** (RTK corrections, **highest priority**): the ground station sends
  RTCM3 to port 4212; the ESP32 writes it verbatim to the GNSS receiver on
  UART1 from its highest-priority executor. See
  [docs/architecture.md](docs/architecture.md#priorities).
- **Uplink** (position commands): The ground station sends raw datagrams to
  the ESP32 on port 4211. The ESP32 COBS-encodes each datagram and writes it
  to UART.

## Hardware

Two boards are supported, selected at build time via a Cargo feature
(`board-waveshare`, the default, or `board-xiao-c6`). Both use the ESP32-C6
MCU and native USB for programming/logging. Only the RTCM pin, status LED, and
antenna handling differ. Each board's wiring lives in its own file under
`src/board/`; to add a board, see [docs/board_info.md](docs/board_info.md#adding-a-board).

| Function | Connects to | Waveshare ESP32-C6-Zero | Seeed XIAO ESP32-C6 |
|----------|-------------|-------------------------|---------------------|
| UART0 TX | STM32 USART6 RX (PC7) | GPIO16 | GPIO16 (D6) |
| UART0 RX | STM32 USART6 TX (PC6) | GPIO17 | GPIO17 (D7) |
| UART1 TX | ublox UART2 (RTCM3 in) | GPIO14 | GPIO0 (D0) |
| Status LED | onboard | WS2812 RGB on GPIO8 | mono LED on GPIO15 |
| Antenna | onboard | fixed | GPIO3=LOW + GPIO14=LOW (built-in PCB); GPIO14=HIGH (external u.FL) with `ext-antenna` |
| GND | STM32 GND | GND | GND |

UART0 configuration: 921600 baud, 8N1, no flow control. UART1: 115200 baud,
TX-only. On the XIAO, GPIO14 is the antenna switch, so the RTCM3 TX moves to
GPIO0.

## LED Status

**Waveshare (WS2812 RGB on GPIO8):**

| Pattern | Meaning |
|---------|---------|
| Breathing blue | Booting / connecting to WiFi |
| Solid dim green | WiFi connected, bridge running |
| Blinking red | WiFi disconnected |

**XIAO (single mono LED on GPIO15 — status by blink rate):**

| Pattern | Meaning |
|---------|---------|
| Slow blink (~1 Hz) | Booting / connecting to WiFi |
| Solid on | WiFi connected, bridge running |
| Fast blink (~5 Hz) | WiFi disconnected |

## Configuration

Two settings are fixed at build time, so rebuild and reflash after changing
either:

| Setting | Where to set it |
|---------|-----------------|
| ESP32 board (and antenna) | Cargo feature on the build command — see [Board](#board) |
| WiFi network name and password | `WIFI_SSID`, `WIFI_PASSWORD` in `src/bin/main.rs` |

The ground station's IP is **not** configured anywhere: the bridge learns it
while running. See [Ground station](#ground-station).

### Board

Choose exactly one board feature when building (commands in
[Building & Flashing](#building--flashing)):

| Board | Features |
|-------|----------|
| Waveshare ESP32-C6-Zero | `board-waveshare` (the default) |
| Seeed XIAO ESP32-C6, built-in antenna | `board-xiao-c6` |
| Seeed XIAO ESP32-C6, external u.FL antenna fitted | `board-xiao-c6,ext-antenna` |

A build with no board or with two boards fails with an error saying so. Only
use `ext-antenna` when an antenna is actually plugged in; see
[docs/board_info.md](docs/board_info.md#xiao-antenna-selection). The boot log
prints which board the firmware was built for.

### WiFi

Edit the constants near the top of `src/bin/main.rs`:

```rust
const WIFI_SSID: &str = "YOUR_SSID";
const WIFI_PASSWORD: &str = "YOUR_PASSWORD";
```

The bridge joins this network and gets its own IP by DHCP. `cybgcs`
identifies each drone by its bridge's IP, so reserve a fixed address for every
bridge in the router and list those addresses in the `cybgcs` fleet config.
These values are compiled in and committed with the source, so everyone with
access to this repo can see the WiFi password.

### Ground station

Nothing to set on the bridge, so any teammate's PC on the same WiFi works
without reflashing. From the moment it starts, `cybgcs` sends every drone a
fleet-state datagram 10 times a second. The bridge takes the sender of the
first datagram it receives on port 4211 or 4212 as its ground station, and
sends telemetry there.

- **Start `cybgcs` to get telemetry.** Until the bridge has heard from a
  ground station it has nowhere to send telemetry and discards it; its log
  shows `Downlink: waiting to hear from the ground station`.
- **One ground station per drone.** The bridge belongs to the first ground
  station it hears and ignores every other PC: their commands, Vicon poses,
  origin and RTK corrections are dropped before they reach the flight
  controller, so a second `cybgcs` started by mistake can't interfere. The
  bridge tells that PC which ground station owns it, about once a second, and
  its `cybgcs` shows the drone as `taken` with the owner's IP. Another PC takes
  over only after the owner has been silent for 3 seconds, for example after
  it quits.
- **The ground station is also the NTP server** (UDP 123), so its PC must run
  one for time sync to work.
- Anything on the WiFi that sends to a bridge's port 4211 or 4212 while it has
  no active ground station becomes its ground station; the WiFi password is
  the only protection.

### Watching from another PC

To see the drones from a second PC, watch through the ground station instead
of starting another one. The drones then send nothing extra:

- **Full dashboard:** `cybgcs tui --observe <ground station IP>`. The ground
  station forwards every drone's telemetry to this PC, which sends nothing to
  the drones (its uplink panes are disabled).
- **Plots only:** `rerun rerun+http://<ground station IP>:9876/proxy` opens the
  ground station's own Rerun stream, without the TUI health dashboard. The
  ground station's firewall must allow TCP 9876.

Either way the copies cross the network: if both PCs are on the drones' WiFi,
they share the drones' channel. Put ground-station PCs on Ethernet where you
can.

## Building & Flashing

Prerequisites:
- Rust stable toolchain with `riscv32imac-unknown-none-elf` target
- `espflash` CLI: `cargo install espflash`

```sh
# --- Waveshare ESP32-C6-Zero (default) ---
cargo build --release
cargo run --release                 # build + flash + monitor

# --- Seeed XIAO ESP32-C6 ---
cargo build --release --no-default-features --features board-xiao-c6
cargo run   --release --no-default-features --features board-xiao-c6

# --- Seeed XIAO ESP32-C6 with an external u.FL antenna fitted ---
cargo run   --release --no-default-features --features board-xiao-c6,ext-antenna
```

Log output appears in the monitor via USB serial. Set verbosity via the
`ESP_LOG` environment variable (default: `info`).

## Protocol Details

### COBS Framing (UART leg)

Each UART frame is COBS-encoded with a `0x00` delimiter:

```
... [COBS-encoded bytes] [0x00] [COBS-encoded bytes] [0x00] ...
```

After COBS decoding, each frame contains:

```
[msg_id: u8] [seq: u8] [payload: N bytes]
```

The bridge treats this as an opaque byte sequence — it does not inspect
`msg_id`, `seq`, or the payload.

### UDP (WiFi leg)

Each downlink datagram carries one UART burst as a batch envelope
(`cybflight_msgs::udp_batch`): a `0x00` marker, a version byte, four bridge
counters (UART errors, telemetry drops, RTCM received, RTCM dropped), then
`[len: u8][msg_id][seq][payload]` per frame. msg_id 0 is reserved, so a
ground station tells a batch from a legacy single-frame datagram by its first
byte. Set `DOWNLINK_BATCHING = false` in `src/bin/main.rs` to send one raw
frame per datagram for a ground station that predates the envelope.

### Ports

| Port | Direction | Description |
|------|-----------|-------------|
| 4210 | ESP32 → Ground | Downlink telemetry |
| 4211 | Ground → ESP32 | Uplink commands (e.g. Vicon pose) |
| 4212 | Ground → ESP32 | RTCM3 corrections for the GNSS receiver |
| 123 | ESP32 → Ground | NTP time sync (the ground station runs the NTP server) |
