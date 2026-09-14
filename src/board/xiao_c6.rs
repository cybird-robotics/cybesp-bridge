//! Seeed XIAO ESP32-C6: single monochrome status LED, RF switch between the
//! built-in PCB antenna and an external u.FL antenna.
//!
//! | Link | UART | Pins |
//! |------|------|------|
//! | STM32 (USART6) | UART0 | TX GPIO16 (D6), RX GPIO17 (D7) |
//! | GNSS receiver (RTCM3 in) | UART1 | TX GPIO0 (D0) — GPIO14 is the antenna switch |
//! | Status LED | — | GPIO15, active-low |
//! | RF switch enable | — | GPIO3, held LOW |
//! | Antenna select | — | GPIO14: LOW built-in, HIGH external (`ext_antenna` setting) |

use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::peripherals::{FLASH, Peripherals};
use esp_hal::uart::Uart;
use static_cell::StaticCell;

use super::{Board, BoardConfig, LinkStatus};

pub const NAME: &str = "Seeed XIAO ESP32-C6";
pub const HAS_ANTENNA_SWITCH: bool = true;

/// The user LED is active-low (driving the pin LOW lights it). Flip these if a
/// board revision wires it active-high.
const LED_ON: Level = Level::Low;
const LED_OFF: Level = Level::High;

pub fn init(p: Peripherals, configure: impl FnOnce(FLASH<'static>) -> BoardConfig) -> Board {
    let cfg = configure(p.FLASH);

    let (fc_rx, fc_tx) = Uart::new(p.UART0, cfg.fc_uart)
        .unwrap()
        .with_tx(p.GPIO16)
        .with_rx(p.GPIO17)
        .split();

    // The GNSS receiver doesn't talk back on this UART, so no RX pin is bound.
    let (_, gnss_tx) = Uart::new(p.UART1, cfg.gnss_uart)
        .unwrap()
        .with_tx(p.GPIO0)
        .split();

    // Enable the RF switch and select the antenna, before WiFi init. GPIO14
    // LOW selects the built-in PCB antenna, HIGH the external u.FL connector;
    // external with no antenna fitted leaves a very weak link (see
    // docs/board_info.md). The drivers live in a static, never dropped, so the
    // pins stay driven for the program's lifetime whatever esp-hal does on drop.
    let antenna = if cfg.ext_antenna {
        Level::High
    } else {
        Level::Low
    };
    static RF_SWITCH: StaticCell<[Output<'static>; 2]> = StaticCell::new();
    RF_SWITCH.init([
        Output::new(p.GPIO3, Level::Low, OutputConfig::default()),
        Output::new(p.GPIO14, antenna, OutputConfig::default()),
    ]);

    let led = StatusLed(Output::new(p.GPIO15, LED_OFF, OutputConfig::default()));

    Board {
        fc_rx,
        fc_tx,
        gnss_tx: Some(gnss_tx),
        led,
        wifi: p.WIFI,
        timg0: p.TIMG0,
        sw_interrupt: p.SW_INTERRUPT,
        usb: p.USB_DEVICE,
    }
}

pub struct StatusLed(Output<'static>);

impl StatusLed {
    /// No color, so status is shown by blink rate. Booting / connecting: slow
    /// blink (~1 Hz). Connected: solid on. Disconnected: fast blink (~5 Hz).
    pub fn show(&mut self, status: LinkStatus, tick: u8) {
        let on = match status {
            LinkStatus::Booting | LinkStatus::WifiConnecting => (tick / 10).is_multiple_of(2),
            LinkStatus::WifiConnected => true,
            LinkStatus::WifiDisconnected => (tick / 2).is_multiple_of(2),
        };
        self.0.set_level(if on { LED_ON } else { LED_OFF });
    }
}
