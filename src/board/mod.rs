//! Board support: everything that differs between the ESP32 boards the bridge
//! runs on — which UART peripheral and pins carry each link, the status LED,
//! and antenna handling.
//!
//! Exactly one `board-*` Cargo feature selects one board file below (enforced
//! by `build.rs`). Every board file provides the same items:
//!
//! - `NAME`: the board's name, logged at boot.
//! - `HAS_ANTENNA_SWITCH`: whether [`BoardConfig::ext_antenna`] means anything.
//! - `init(Peripherals, configure) -> Board`: first hands the flash peripheral
//!   to `configure`, which returns the [`BoardConfig`] (the settings live in
//!   flash); then claims the board's UARTs and pins, applies board-level setup
//!   such as the antenna switch, and hands back the peripherals the
//!   application still needs.
//! - `StatusLed`, with `show(LinkStatus, tick)` called every [`LED_TICK_MS`].
//!
//! The application never names a GPIO or a UART instance; it uses the links
//! in [`Board`] by role. esp-hal's UART driver types do not depend on the
//! instance or the pins, so every board returns the same types.
//!
//! Adding a board: see `docs/board_info.md`.

#[cfg_attr(feature = "board-waveshare", path = "waveshare_c6_zero.rs")]
#[cfg_attr(feature = "board-xiao-c6", path = "xiao_c6.rs")]
mod imp;

pub use imp::{HAS_ANTENNA_SWITCH, NAME, StatusLed, init};

use esp_hal::Blocking;
use esp_hal::peripherals::{SW_INTERRUPT, TIMG0, USB_DEVICE, WIFI};
use esp_hal::uart::{self, UartRx, UartTx};

/// Period of `StatusLed::show` calls; LED patterns are counted in these ticks.
pub const LED_TICK_MS: u64 = 50;

/// Settings chosen by the application. A board decides only which UART and
/// pins carry each link, never its baud rate or FIFO settings.
pub struct BoardConfig {
    /// STM32 flight-controller link: bidirectional, COBS framed.
    pub fc_uart: uart::Config,
    /// RTCM3 corrections to the GNSS receiver: TX only, unframed.
    pub gnss_uart: uart::Config,
    /// Use the external antenna connector. Ignored by a board without an
    /// antenna switch (the settings refuse it there).
    pub ext_antenna: bool,
}

/// What a board hands to the application. The UART halves are Blocking; each
/// task that owns one converts it to Async on its own executor.
pub struct Board {
    pub fc_rx: UartRx<'static, Blocking>,
    pub fc_tx: UartTx<'static, Blocking>,
    /// `None` on a board with no UART or pin to spare for RTCM corrections;
    /// the application then does not start the RTCM path.
    pub gnss_tx: Option<UartTx<'static, Blocking>>,
    pub led: StatusLed,
    // Peripherals the application needs that no board uses.
    pub wifi: WIFI<'static>,
    pub timg0: TIMG0<'static>,
    pub sw_interrupt: SW_INTERRUPT<'static>,
    /// The USB-Serial-JTAG port, for the settings shell.
    pub usb: USB_DEVICE<'static>,
}

/// Bridge state shown on the status LED.
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LinkStatus {
    Booting = 0,
    WifiConnecting = 1,
    WifiConnected = 2,
    WifiDisconnected = 3,
}

impl LinkStatus {
    /// Inverse of `as u8`, for state shared through an `AtomicU8`.
    pub fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::WifiConnecting,
            2 => Self::WifiConnected,
            3 => Self::WifiDisconnected,
            _ => Self::Booting,
        }
    }
}
