
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
| Antenna select | GPIO14 | — | LOW = built-in PCB antenna (default); HIGH = external u.FL with the `ext-antenna` feature |

Key differences from the Waveshare board:
- No RGB LED — status is shown by blink rate on the single GPIO15 LED.
- GPIO14 is the antenna switch, so RTCM3 TX moves to GPIO0.
- WiFi requires explicit antenna setup: GPIO3 LOW, and GPIO14 LOW for the PCB
  antenna or HIGH for the external one (`ext-antenna`).

## XIAO antenna selection

The XIAO has an RF switch that routes the radio to either the built-in PCB
(ceramic) antenna or the external u.FL connector. The firmware sets it once at
boot, in `init()` in `src/board/xiao_c6.rs`, according to the `ext-antenna`
Cargo feature:

| GPIO3 (RF switch enable) | GPIO14 (antenna select) | Antenna used | Features |
|--------------------------|-------------------------|--------------|----------|
| LOW | LOW | built-in PCB antenna (**default**) | `board-xiao-c6` |
| LOW | HIGH | external u.FL antenna | `board-xiao-c6,ext-antenna` |

**The GPIO14 level must match the hardware.** Selecting the external antenna
with nothing plugged into the u.FL connector sends the radio into an empty
connector, and the WiFi link becomes very weak. Selecting the built-in antenna
while an external one is fitted just ignores the external one.

### Switching to an external antenna

1. **Fit the antenna first.** Use a 2.4 GHz antenna with a u.FL (IPEX-1)
   plug, and fit it with the board unpowered. u.FL connectors are fragile and
   rated for only a few dozen mating cycles, so press straight down and pull
   by the plug, not the cable.
2. **Build and flash with the `ext-antenna` feature**, which drives GPIO14
   HIGH (`ANTENNA_SELECT` in `src/board/xiao_c6.rs`); no code change needed:
   `cargo run --release --no-default-features --features board-xiao-c6,ext-antenna`
   The boot log then names the board "Seeed XIAO ESP32-C6 (external antenna)".
3. **Check the link before flight.** Compare the link on the bench and at the
   distance you will fly: the status LED staying solid, and `cybgcs` showing
   steady downlink with no rising drop counters. If the link is worse than
   with the built-in antenna, check that the plug is seated and that the boot
   log shows the external-antenna build.
4. **Label the board** (for example "EXT ANT") so it isn't later flashed with
   a build that selects the built-in antenna, or vice versa.

The Waveshare ESP32-C6-Zero has only its onboard antenna and no switch, so
none of this applies to it; `ext-antenna` is a compile error there.


# Adding a board

Everything board-specific lives in one file under `src/board/`; the
application code in `src/bin/main.rs` doesn't change.

1. **Create `src/board/<board>.rs`**, starting from the closest existing
   board file. It provides `NAME`, `init(Peripherals, LinkConfig) -> Board`
   and `StatusLed::show(LinkStatus, tick)`. `init` picks the UART instance and
   pins for the STM32 link and for the GNSS UART; the baud rates and FIFO
   settings arrive in `LinkConfig`, so use them as given. Return
   `gnss_tx: None` if the board has no UART or pin to spare for RTCM.
2. **Add its feature** in `Cargo.toml` (`board-<name> = [...]`, with any
   board-only dependencies as `dep:`), and a matching `#[cfg_attr(feature =
   ..., path = ...)]` line in `src/board/mod.rs`. `build.rs` needs no change.
3. **Reject options the board can't support** with a `compile_error!` in the
   board file, as `waveshare_c6_zero.rs` does for `ext-antenna`.
4. **Add it to the CI matrix** in `.github/workflows/rust.yml`.
5. **Document it:** a section in this file with its pin table and quirks, and
   a column in the README hardware table.

A board with a different ESP32 chip needs more: the chip is also named in the
esp-* dependencies in `Cargo.toml`, in `rust-toolchain.toml` (target), and in
the `espflash` runner in `.cargo/config.toml`.

## Choosing pins

Any GPIO can carry any UART signal through the GPIO matrix, but avoid:

- **Strapping pins.** They are sampled at reset to choose the boot mode, so an
  external device holding one high or low can stop the chip booting. On the
  C6 these include GPIO8, GPIO9 (BOOT button) and GPIO15; check the chip
  datasheet for the full list.
- **The USB pins** (GPIO12/13 on the C6), needed for flashing and logging.
- **The SPI flash pins**, and pins the board already uses, such as the XIAO's
  GPIO3/GPIO14 antenna switch.
- **UART0's default pins** (GPIO16/17 on the C6) where you can. The ROM prints
  its boot messages there at reset, so whatever is connected receives them.
  Both current boards put the STM32 link there, which relies on the STM32
  discarding that text.
