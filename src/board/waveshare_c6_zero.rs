//! Waveshare ESP32-C6-Zero: WS2812 RGB status LED, onboard antenna only.
//!
//! | Link | UART | Pins |
//! |------|------|------|
//! | STM32 (USART6) | UART0 | TX GPIO16, RX GPIO17 |
//! | GNSS receiver (RTCM3 in) | UART1 | TX GPIO14 |
//! | Status LED | RMT ch 0 | GPIO8 (WS2812, RGB wire order) |

#[cfg(feature = "ext-antenna")]
compile_error!("ext-antenna: the Waveshare ESP32-C6-Zero has no external antenna connector");

use esp_hal::peripherals::Peripherals;
use esp_hal::rmt::{PulseCode, Rmt};
use esp_hal::uart::Uart;
use esp_hal_smartled::SmartLedsAdapter;
use smart_leds::SmartLedsWrite;
use static_cell::StaticCell;

use super::{Board, LinkConfig, LinkStatus};

pub const NAME: &str = "Waveshare ESP32-C6-Zero";

/// RMT buffer size for 1 WS2812 LED (24 bits + 1 reset pulse).
const LED_RMT_BUFFER_SIZE: usize = 25;

pub fn init(p: Peripherals, cfg: LinkConfig) -> Board {
    let (fc_rx, fc_tx) = Uart::new(p.UART0, cfg.fc_uart)
        .unwrap()
        .with_tx(p.GPIO16)
        .with_rx(p.GPIO17)
        .split();

    // The GNSS receiver doesn't talk back on this UART, so no RX pin is bound.
    let (_, gnss_tx) = Uart::new(p.UART1, cfg.gnss_uart)
        .unwrap()
        .with_tx(p.GPIO14)
        .split();

    let rmt = Rmt::new(p.RMT, esp_hal::time::Rate::from_mhz(80)).unwrap();
    static LED_BUF: StaticCell<[PulseCode; LED_RMT_BUFFER_SIZE]> = StaticCell::new();
    let rmt_buf = LED_BUF.init([PulseCode::default(); LED_RMT_BUFFER_SIZE]);
    let led = StatusLed(SmartLedsAdapter::new(rmt.channel0, p.GPIO8, rmt_buf));

    Board {
        fc_rx,
        fc_tx,
        gnss_tx: Some(gnss_tx),
        led,
        wifi: p.WIFI,
        timg0: p.TIMG0,
        sw_interrupt: p.SW_INTERRUPT,
    }
}

pub struct StatusLed(SmartLedsAdapter<'static, LED_RMT_BUFFER_SIZE>);

impl StatusLed {
    /// Booting / connecting: breathing blue. Connected: solid dim green.
    /// Disconnected: blinking red.
    pub fn show(&mut self, status: LinkStatus, tick: u8) {
        // The onboard LED uses RGB wire order (not GRB). SmartLedsAdapter
        // sends Grb fields [g, r, b] in memory order, so the intended R goes
        // in the g field and G in the r field.
        let color = match status {
            LinkStatus::Booting => rgb::Grb {
                g: 0,
                r: 0,
                b: triangle_wave(tick, 40),
            },
            LinkStatus::WifiConnecting => rgb::Grb {
                g: 0,
                r: 0,
                b: triangle_wave(tick, 30),
            },
            // Dim green: green value in the r field (2nd byte = G on the wire).
            LinkStatus::WifiConnected => rgb::Grb { g: 0, r: 15, b: 0 },
            // Red in the g field (1st byte = R on the wire), 250 ms on/off.
            LinkStatus::WifiDisconnected => {
                let on = (tick / 5).is_multiple_of(2);
                rgb::Grb {
                    g: if on { 40 } else { 0 },
                    r: 0,
                    b: 0,
                }
            }
        };
        let _ = self.0.write([color]);
    }
}

/// Triangle wave: ramps 0 → `max_brightness` → 0 over 100 ticks (5 s).
fn triangle_wave(tick: u8, max_brightness: u8) -> u8 {
    let phase = tick % 100;
    let ramp = if phase < 50 { phase } else { 100 - phase };
    ((ramp as u16 * max_brightness as u16) / 50) as u8
}
