//! Log output on the USB-Serial-JTAG port: one `LEVEL - message` line per
//! record, like esp-println's logger, but ended in CR LF. The settings shell
//! shares the port, and a terminal such as minicom needs the CR to start the
//! next line at the left margin; esp-println ends lines with a bare LF, which
//! only `espflash monitor` translates.
//!
//! The level comes from `ESP_LOG` at build time (`.cargo/config.toml` sets
//! `info`): `off`, `error`, `warn`, `info`, `debug` or `trace`. Per-module
//! filters such as `ESP_LOG=info,foo=debug` are not supported.

use log::{LevelFilter, Log, Metadata, Record};

struct CrlfLogger;

impl Log for CrlfLogger {
    fn enabled(&self, _: &Metadata) -> bool {
        true // filtered by `log`'s max level already
    }

    fn log(&self, record: &Record) {
        esp_println::print!("{} - {}\r\n", record.level(), record.args());
    }

    fn flush(&self) {}
}

/// Install the logger at the `ESP_LOG` level (default `info`).
pub fn init() {
    static LOGGER: CrlfLogger = CrlfLogger;
    let level = match option_env!("ESP_LOG").unwrap_or("info") {
        "off" => LevelFilter::Off,
        "error" => LevelFilter::Error,
        "warn" => LevelFilter::Warn,
        "debug" => LevelFilter::Debug,
        "trace" => LevelFilter::Trace,
        _ => LevelFilter::Info,
    };
    if log::set_logger(&LOGGER).is_ok() {
        log::set_max_level(level);
    }
}
