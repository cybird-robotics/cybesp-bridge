//! A line-based shell on the USB-Serial-JTAG port, the one the logs use, for
//! reading and changing the bridge settings (`crate::settings`). Open the
//! port in a terminal (for example `minicom -D /dev/ttyACM0`, hardware flow
//! control off) and type `help`. The commands follow the flight controller's
//! shell: `param list / get / set / save / reset`, then `reboot` to apply.
//!
//! Input comes through esp-hal's USB-Serial-JTAG driver, receive half only.
//! All output, the echo included, goes through `esp_println` like the logs:
//! esp-println writes the port's transmit FIFO directly, so a second writer
//! (the driver's transmit half) would interleave with it byte by byte.

use embassy_net::Stack;
use esp_hal::peripherals::USB_DEVICE;
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use esp_println::print;

use crate::settings::{self, PARAMS, Param, Store};

/// `println!` ending the line in CR LF, like the log lines (see
/// `crate::logger`): a terminal such as minicom needs the CR to start the next
/// line at the left margin.
macro_rules! outln {
    () => {
        print!("\r\n")
    };
    ($($arg:tt)*) => {
        print!("{}\r\n", format_args!($($arg)*))
    };
}

/// Longest command line; characters past it are dropped.
const LINE_MAX: usize = 128;

/// What the shell needs from the application besides the settings.
pub struct Hooks {
    /// The WiFi network stack, for its address (`ip`).
    pub net: Stack<'static>,
    /// Print the bridge's link and counters (`stats`).
    pub print_stats: fn(),
    /// Turn the periodic counters log on or off (`stats on|off`).
    pub set_stats_log: fn(bool),
}

/// Read and run commands for as long as the bridge runs.
pub async fn run(usb: USB_DEVICE<'static>, mut store: Store, hooks: Hooks) -> ! {
    // The transmit half stays unused but alive (see the module docs).
    let (mut rx, _tx) = UsbSerialJtag::new(usb).into_async().split();
    let mut line = [0u8; LINE_MAX];
    let mut len = 0;
    let mut after_cr = false;
    let mut chunk = [0u8; 64];
    loop {
        // The USB-Serial-JTAG receiver can't fail (its error type is empty).
        let Ok(n) = embedded_io_async::Read::read(&mut rx, &mut chunk).await;
        for &b in &chunk[..n] {
            match b {
                // CR LF: the CR already ended the line.
                b'\n' if after_cr => {}
                b'\r' | b'\n' => {
                    outln!();
                    // Only printable ASCII is ever stored in `line`.
                    let text = core::str::from_utf8(&line[..len]).unwrap_or("");
                    execute(text.trim(), &mut store, &hooks).await;
                    len = 0;
                    print!("> ");
                }
                0x08 | 0x7f if len > 0 => {
                    len -= 1;
                    print!("\x08 \x08");
                }
                0x20..=0x7e if len < LINE_MAX => {
                    line[len] = b;
                    len += 1;
                    print!("{}", b as char);
                }
                _ => {}
            }
            after_cr = b == b'\r';
        }
    }
}

async fn execute(line: &str, store: &mut Store, hooks: &Hooks) {
    let (cmd, args) = split_word(line);
    match cmd {
        "" => {}
        "help" => help(),
        "param" => param(args, store).await,
        "ip" => ip(hooks.net),
        "stats" => match args {
            "" => (hooks.print_stats)(),
            "on" => {
                (hooks.set_stats_log)(true);
                outln!("counters log: on");
            }
            "off" => {
                (hooks.set_stats_log)(false);
                outln!("counters log: off");
            }
            _ => outln!("usage: stats | stats on | stats off"),
        },
        "reboot" => {
            if store.any_unsaved() {
                outln!("note: unsaved changes are lost");
            }
            outln!("rebooting");
            esp_hal::system::software_reset();
        }
        _ => outln!("unknown command `{cmd}`; type `help`"),
    }
}

async fn param(args: &str, store: &mut Store) {
    let (sub, rest) = split_word(args);
    match sub {
        "" | "list" => list(store),
        "get" => match settings::find(rest) {
            Some(p) => show(store, p),
            None => unknown(rest),
        },
        "set" => {
            let (name, value) = split_word(rest);
            let Some(p) = settings::find(name) else {
                return unknown(name);
            };
            match store.current.set(p, strip_quotes(value)) {
                Ok(()) => {
                    show(store, p);
                    outln!("not saved yet: `param save`, then `reboot` to apply");
                }
                Err(e) => outln!("{}: {e}", p.name),
            }
        }
        "save" => {
            if !store.any_unsaved() {
                outln!("nothing to save");
                return;
            }
            outln!("saving (telemetry pauses briefly while flash is written)");
            match store.save().await {
                Ok(n) => outln!("saved {n} setting(s); `reboot` to apply"),
                Err(e) => outln!("save failed: {e}"),
            }
        }
        "reset" if rest == "all" => {
            for p in &PARAMS {
                store.reset(p);
            }
            list(store);
        }
        "reset" => match settings::find(rest) {
            Some(p) => {
                store.reset(p);
                show(store, p);
            }
            None => unknown(rest),
        },
        _ => {
            outln!("usage: param list | get <name> | set <name> <value> | save | reset <name>|all")
        }
    }
}

/// The bridge's address on the WiFi network (`ip`), as DHCP assigned it.
fn ip(net: Stack<'static>) {
    outln!("mac      {}", net.hardware_address());
    if !net.is_link_up() {
        outln!("ip       none (WiFi not connected)");
        return;
    }
    let Some(cfg) = net.config_v4() else {
        outln!("ip       none (connected, waiting for DHCP)");
        return;
    };
    outln!("ip       {}", cfg.address);
    match cfg.gateway {
        Some(gw) => outln!("gateway  {gw}"),
        None => outln!("gateway  none"),
    }
    if cfg.dns_servers.is_empty() {
        outln!("dns      none");
    }
    for dns in &cfg.dns_servers {
        outln!("dns      {dns}");
    }
}

fn list(store: &Store) {
    for p in &PARAMS {
        show(store, p);
    }
    if store.any_unsaved() {
        outln!("unsaved changes: `param save`, then `reboot` to apply");
    }
    if !store.can_save() {
        outln!("(no settings partition on this board: changes can't be saved)");
    }
}

fn show(store: &Store, p: &Param) {
    let tag = if store.unsaved(p) { "  [unsaved]" } else { "" };
    outln!("{:<11} = {}{tag}", p.name, store.current.display(p));
}

fn unknown(name: &str) {
    outln!("no setting `{name}`; `param list` shows them");
}

fn help() {
    outln!("commands:");
    outln!("  param list                 show every setting");
    outln!("  param get <name>           show one");
    outln!("  param set <name> <value>   the value is the rest of the line; quotes optional");
    outln!("  param save                 write changes to flash");
    outln!("  param reset <name>|all     back to the built-in default (then save)");
    outln!("  reboot                     restart and apply the saved settings");
    outln!("  ip                         show the bridge's address on the WiFi network");
    outln!(
        "  stats                      show the link and counters (peaks since the last `stats`)"
    );
    outln!("  stats on|off               log the counters every few seconds (off at boot)");
    outln!("settings:");
    for p in &PARAMS {
        outln!("  {:<11}  {}", p.name, p.help);
    }
}

/// The first word of `s` and the rest, both trimmed.
fn split_word(s: &str) -> (&str, &str) {
    let s = s.trim();
    match s.find(char::is_whitespace) {
        Some(i) => (&s[..i], s[i..].trim_start()),
        None => (s, ""),
    }
}

fn strip_quotes(value: &str) -> &str {
    value
        .strip_prefix('"')
        .and_then(|v| v.strip_suffix('"'))
        .unwrap_or(value)
}
