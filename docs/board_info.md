
Both boards use the ESP32-C6 MCU. The target is selected at build time via a
Cargo feature: `board-waveshare` (default) or `board-xiao-c6`. See README for
build commands.

# Current Board (WAVESHARE-ESP32-C6)

- information: https://docs.waveshare.com/ESP32-C6-Zero?variant=ESP32-C6-Zero
- microcomputer: ESP32-C6
- feature: `board-waveshare` (default)

| Function | GPIO | Notes |
|----------|------|-------|
| UART0 TX → STM32 | GPIO16 | 921600 baud, COBS, bidirectional |
| UART0 RX ← STM32 | GPIO17 | |
| UART1 TX → ublox (RTCM3) | GPIO14 | 115200 baud, TX-only |
| Status LED | GPIO8 | WS2812 RGB via RMT |
| Antenna | — | onboard, no GPIO control |


# New Board (SEEED XIAO ESP32-C6)

- information: https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/#resources
- microcomputer: ESP32-C6
- feature: `board-xiao-c6`

| Function | GPIO | XIAO pin | Notes |
|----------|------|----------|-------|
| UART0 TX → STM32 | GPIO16 | D6 | 921600 baud, COBS, bidirectional |
| UART0 RX ← STM32 | GPIO17 | D7 | |
| UART1 TX → ublox (RTCM3) | GPIO0 | D0 | 115200 baud, TX-only (GPIO14 unavailable — antenna switch) |
| Status LED | GPIO15 | — | single monochrome LED, active-low, blink-rate status |
| RF switch enable | GPIO3 | — | held LOW |
| Antenna select | GPIO14 | — | held LOW = built-in PCB antenna (HIGH = external u.FL) |

Key differences from the Waveshare board:
- No RGB LED — status is shown by blink rate on the single GPIO15 LED.
- GPIO14 is the antenna switch, so RTCM3 TX moves to GPIO0.
- WiFi requires explicit antenna setup: GPIO3 LOW + GPIO14 LOW for the PCB antenna.
