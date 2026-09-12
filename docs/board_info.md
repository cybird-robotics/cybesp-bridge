
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

## XIAO antenna selection

The XIAO has an RF switch that routes the radio to either the built-in PCB
(ceramic) antenna or the external u.FL connector. The firmware sets it once at
boot, in the `6c` block of `main()` in `src/bin/main.rs`:

| GPIO3 (RF switch enable) | GPIO14 (antenna select) | Antenna used |
|--------------------------|-------------------------|--------------|
| LOW | LOW | built-in PCB antenna (**default**) |
| LOW | HIGH | external u.FL antenna |

**The GPIO14 level must match the hardware.** Selecting the external antenna
with nothing plugged into the u.FL connector sends the radio into an empty
connector, and the WiFi link becomes very weak. Selecting the built-in antenna
while an external one is fitted just ignores the external one.

### Switching to an external antenna

1. **Fit the antenna first.** Use a 2.4 GHz antenna with a u.FL (IPEX-1)
   plug, and fit it with the board unpowered. u.FL connectors are fragile and
   rated for only a few dozen mating cycles, so press straight down and pull
   by the plug, not the cable.
2. **Change the firmware.** In `src/bin/main.rs`, on the `_ant_select` line in
   the `6c` block, change `Level::Low` to `Level::High`. Leave `_rf_enable`
   (GPIO3) LOW. Update the comment above it, the Antenna select row in the
   tables above, and the matching rows in `docs/architecture.md` and
   `README.md`.
3. **Build and flash the XIAO build:**
   `cargo run --release --no-default-features --features board-xiao-c6`
4. **Check the link before flight.** Compare the link on the bench and at the
   distance you will fly: the status LED staying solid, and `cybgcs` showing
   steady downlink with no rising drop counters. If the link is worse than
   with the built-in antenna, check that the plug is seated and that GPIO14
   really is HIGH in the flashed build.
5. **Label the board** (for example "EXT ANT") so it isn't later flashed with
   a build that selects the built-in antenna, or vice versa.

The Waveshare ESP32-C6-Zero has only its onboard antenna and no switch, so
none of this applies to it.
