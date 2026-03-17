# cybesp-bridge

Message-agnostic WiFi-UART bridge for the cybflight flight controller.

Runs on a **Waveshare ESP32-C6-Zero** and forwards COBS-framed UART data from
an STM32H743 to raw UDP datagrams over WiFi, and vice versa. The bridge does
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
  921600 baud. The ESP32 decodes COBS framing and sends each decoded frame as
  a UDP datagram to the ground station on port 4210.
- **Uplink** (position commands): The ground station sends raw datagrams to
  the ESP32 on port 4211. The ESP32 COBS-encodes each datagram and writes it
  to UART.

## Hardware

| Pin | Function | Connects to |
|-----|----------|-------------|
| GPIO16 | UART0 TX | STM32 USART6 RX (PC7) |
| GPIO17 | UART0 RX | STM32 USART6 TX (PC6) |
| GND | Ground | STM32 GND |

UART configuration: 921600 baud, 8N1, no flow control.

The ESP32-C6-Zero uses native USB for programming and log output, so UART0
pins are free for external communication.

## LED Status (WS2812 on GPIO8)

| Pattern | Meaning |
|---------|---------|
| Breathing blue | Booting / connecting to WiFi |
| Solid dim green | WiFi connected, bridge running |
| Blinking red | WiFi disconnected |

## Configuration

Edit the constants in `src/bin/main.rs`:

```rust
const WIFI_SSID: &str = "TODO_YOUR_SSID";
const WIFI_PASSWORD: &str = "TODO_YOUR_PASSWORD";
const GROUND_STATION_IP: Ipv4Address = Ipv4Address::new(192, 168, 1, 100);
```

Both the ESP32 and ground station must be on the same WiFi network. The ESP32
obtains its IP via DHCP.

## Building & Flashing

Prerequisites:
- Rust stable toolchain with `riscv32imac-unknown-none-elf` target
- `espflash` CLI: `cargo install espflash`

```sh
# Build
cargo build --release

# Build + flash + monitor (USB cable to ESP32-C6-Zero)
cargo run --release
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

Each UDP datagram carries exactly one decoded frame (no COBS, no delimiter).
The datagram payload is the raw `[msg_id][seq][payload]` bytes.

### Ports

| Port | Direction | Description |
|------|-----------|-------------|
| 4210 | ESP32 → Ground | Downlink telemetry |
| 4211 | Ground → ESP32 | Uplink commands (e.g. Vicon pose) |
