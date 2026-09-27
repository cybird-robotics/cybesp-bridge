//! Bridge settings: WiFi credentials, link baud rates and the antenna.
//!
//! Each setting has a built-in default and can be changed from the USB shell
//! (`crate::shell`: `param list / get / set / save / reset`), which stores it
//! in the `nvs` flash partition. At boot the stored values replace the
//! defaults. A missing or invalid stored value falls back to its default, so a
//! new board, or one whose store is blank or corrupt, still starts with the
//! built-in values. Changes take effect at the next boot: the WiFi task and the
//! UARTs read the settings once, at startup.
//!
//! Storage: `esp-storage` for raw flash access, the partition table to find
//! the `nvs` data partition, and a `sequential-storage` map inside it keyed by
//! each setting's id. A value is stored as the same text `param set` takes, so
//! loading runs it through the same checks. Every flash write or erase runs
//! with interrupts masked, pausing the UART and WiFi paths, so the store is
//! written only by an explicit `param save`.

use core::fmt::Write as _;

use embassy_embedded_hal::adapter::BlockingAsync;
use esp_bootloader_esp_idf::partitions::{
    self, DataPartitionSubType, FlashRegion, PARTITION_TABLE_MAX_LEN, PartitionType,
};
use esp_hal::peripherals::FLASH;
use esp_storage::FlashStorage;
use heapless::String;
use log::{info, warn};
use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::map::{MapConfig, MapStorage};
use static_cell::StaticCell;

/// WiFi network joined when no SSID is stored.
const DEFAULT_WIFI_SSID: &str = "";
/// Password for [`DEFAULT_WIFI_SSID`].
const DEFAULT_WIFI_PASS: &str = "";
/// STM32 link baud (8N1, no flow control); must match the flight controller.
const DEFAULT_FC_BAUD: u32 = 921_600;
/// GNSS UART baud (TX only); must match the receiver's RTCM input port.
const DEFAULT_GNSS_BAUD: u32 = 115_200;

/// Longest SSID WiFi allows.
pub const SSID_MAX: usize = 32;
/// Longest WPA2 passphrase (64 characters is a raw hex key).
pub const PASS_MAX: usize = 64;
/// Baud rates the ESP32-C6 UARTs can produce.
const BAUD_RANGE: core::ops::RangeInclusive<u32> = 1_200..=5_000_000;

pub type Ssid = String<SSID_MAX>;
pub type Password = String<PASS_MAX>;

/// Flash map key of each setting. Never reuse a number for another setting:
/// a board keeps its stored values across firmware updates.
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum Key {
    WifiSsid = 1,
    WifiPass = 2,
    FcBaud = 3,
    GnssBaud = 4,
    ExtAntenna = 5,
}

/// One setting, as the shell names and describes it.
pub struct Param {
    pub name: &'static str,
    key: Key,
    pub help: &'static str,
}

pub const PARAMS: [Param; 5] = [
    Param {
        name: "wifi_ssid",
        key: Key::WifiSsid,
        help: "WiFi network to join (1-32 characters)",
    },
    Param {
        name: "wifi_pass",
        key: Key::WifiPass,
        help: "WiFi password (8-64 characters; empty for an open network)",
    },
    Param {
        name: "fc_baud",
        key: Key::FcBaud,
        help: "STM32 link baud; must match the flight controller",
    },
    Param {
        name: "gnss_baud",
        key: Key::GnssBaud,
        help: "GNSS receiver baud for RTCM corrections",
    },
    Param {
        name: "ext_antenna",
        key: Key::ExtAntenna,
        help: "1 = external u.FL antenna (only with one fitted), 0 = built-in",
    },
];

/// The setting called `name`.
pub fn find(name: &str) -> Option<&'static Param> {
    PARAMS.iter().find(|p| p.name == name)
}

/// Every setting's value.
#[derive(Clone, PartialEq, Eq)]
pub struct Settings {
    pub wifi_ssid: Ssid,
    pub wifi_pass: Password,
    pub fc_baud: u32,
    pub gnss_baud: u32,
    /// Use the external antenna connector (boards with an antenna switch).
    pub ext_antenna: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            wifi_ssid: text(DEFAULT_WIFI_SSID).unwrap(),
            wifi_pass: text(DEFAULT_WIFI_PASS).unwrap(),
            fc_baud: DEFAULT_FC_BAUD,
            gnss_baud: DEFAULT_GNSS_BAUD,
            ext_antenna: false,
        }
    }
}

impl Settings {
    /// Set `p` from the text a user typed (or that was stored), checking it.
    pub fn set(&mut self, p: &Param, value: &str) -> Result<(), &'static str> {
        match p.key {
            Key::WifiSsid => {
                if value.is_empty() || value.len() > SSID_MAX {
                    return Err("must be 1-32 characters");
                }
                self.wifi_ssid = text(value)?;
            }
            Key::WifiPass => {
                if !value.is_empty() && !(8..=PASS_MAX).contains(&value.len()) {
                    return Err("must be 8-64 characters, or empty for an open network");
                }
                self.wifi_pass = text(value)?;
            }
            Key::FcBaud => self.fc_baud = baud(value)?,
            Key::GnssBaud => self.gnss_baud = baud(value)?,
            Key::ExtAntenna => {
                if !crate::board::HAS_ANTENNA_SWITCH {
                    return Err("this board has no antenna switch");
                }
                self.ext_antenna = match value {
                    "1" | "true" | "on" | "yes" => true,
                    "0" | "false" | "off" | "no" => false,
                    _ => return Err("use 1 or 0"),
                };
            }
        }
        Ok(())
    }

    /// `p`'s value as the shell and the boot log show it: the password
    /// appears only as its length.
    pub fn display(&self, p: &Param) -> String<80> {
        let mut s = String::new();
        let _ = match p.key {
            Key::WifiPass if self.wifi_pass.is_empty() => write!(s, "(none)"),
            Key::WifiPass => write!(s, "******** ({} characters)", self.wifi_pass.len()),
            _ => write!(s, "{}", self.raw(p)),
        };
        s
    }

    /// `p`'s value as `set` takes it, and as it is stored.
    fn raw(&self, p: &Param) -> Password {
        let mut s = String::new();
        let _ = match p.key {
            Key::WifiSsid => write!(s, "{}", self.wifi_ssid),
            Key::WifiPass => write!(s, "{}", self.wifi_pass),
            Key::FcBaud => write!(s, "{}", self.fc_baud),
            Key::GnssBaud => write!(s, "{}", self.gnss_baud),
            Key::ExtAntenna => write!(s, "{}", u8::from(self.ext_antenna)),
        };
        s
    }

    fn differs(&self, other: &Settings, p: &Param) -> bool {
        match p.key {
            Key::WifiSsid => self.wifi_ssid != other.wifi_ssid,
            Key::WifiPass => self.wifi_pass != other.wifi_pass,
            Key::FcBaud => self.fc_baud != other.fc_baud,
            Key::GnssBaud => self.gnss_baud != other.gnss_baud,
            Key::ExtAntenna => self.ext_antenna != other.ext_antenna,
        }
    }

    fn copy(&mut self, from: &Settings, p: &Param) {
        match p.key {
            Key::WifiSsid => self.wifi_ssid = from.wifi_ssid.clone(),
            Key::WifiPass => self.wifi_pass = from.wifi_pass.clone(),
            Key::FcBaud => self.fc_baud = from.fc_baud,
            Key::GnssBaud => self.gnss_baud = from.gnss_baud,
            Key::ExtAntenna => self.ext_antenna = from.ext_antenna,
        }
    }
}

fn text<const N: usize>(value: &str) -> Result<String<N>, &'static str> {
    let mut s = String::new();
    s.push_str(value).map_err(|_| "too long")?;
    Ok(s)
}

fn baud(value: &str) -> Result<u32, &'static str> {
    match value.parse::<u32>() {
        Ok(b) if BAUD_RANGE.contains(&b) => Ok(b),
        Ok(_) => Err("must be 1200-5000000"),
        Err(_) => Err("not a number"),
    }
}

type Flash = BlockingAsync<FlashRegion<'static, FlashStorage<'static>>>;
/// A handful of small items read once at boot: no cache needed.
type Map = MapStorage<u8, Flash, Cache<Uncached, Uncached, Uncached, u8>>;

/// Room for one map item: the key, the longest value and the map's own
/// framing, rounded up to the flash word.
const ITEM_BUF: usize = 128;

/// The settings as edited (`current`) and as saved in flash.
pub struct Store {
    /// `None` without a usable `nvs` partition: defaults only, no saving.
    map: Option<Map>,
    /// Edited by the shell; used at the next boot once saved.
    pub current: Settings,
    /// What flash holds, with defaults where it holds nothing.
    saved: Settings,
}

impl Store {
    /// Open the store on the `nvs` partition and load it. Never fails: without
    /// a usable partition every setting keeps its default.
    pub async fn open(flash: FLASH<'static>) -> Self {
        let mut store = Self {
            map: open_map(flash),
            current: Settings::default(),
            saved: Settings::default(),
        };
        store.load().await;
        store
    }

    async fn load(&mut self) {
        let Some(map) = self.map.as_mut() else { return };
        let mut buf = [0u8; ITEM_BUF];
        for p in &PARAMS {
            match map.fetch_item::<&[u8]>(&mut buf, &(p.key as u8)).await {
                Ok(None) => {}
                Ok(Some(bytes)) => {
                    let valid = core::str::from_utf8(bytes)
                        .map_err(|_| "not text")
                        .and_then(|v| self.saved.set(p, v));
                    if let Err(e) = valid {
                        warn!(
                            "settings: stored {} is invalid ({e}); using its default",
                            p.name
                        );
                    }
                }
                Err(e) => warn!("settings: can't read {} ({e:?}); using its default", p.name),
            }
        }
        self.current = self.saved.clone();
    }

    /// Write every setting changed since the last save; returns how many. A
    /// setting back at its default is removed rather than stored. Pauses the
    /// bridge briefly for each flash write or erase.
    pub async fn save(&mut self) -> Result<usize, &'static str> {
        let Some(map) = self.map.as_mut() else {
            return Err("no settings partition on this board's flash");
        };
        let defaults = Settings::default();
        let mut buf = [0u8; ITEM_BUF];
        let mut written = 0;
        for p in &PARAMS {
            if !self.current.differs(&self.saved, p) {
                continue;
            }
            let key = p.key as u8;
            let result = if self.current.differs(&defaults, p) {
                let value = self.current.raw(p);
                map.store_item(&mut buf, &key, &value.as_bytes()).await
            } else {
                map.remove_item(&mut buf, &key).await
            };
            if let Err(e) = result {
                warn!("settings: saving {} failed: {e:?}", p.name);
                return Err("flash write failed (see the log line above)");
            }
            self.saved.copy(&self.current, p);
            written += 1;
        }
        Ok(written)
    }

    /// Put `p` back to its built-in default (unsaved until `save`).
    pub fn reset(&mut self, p: &Param) {
        self.current.copy(&Settings::default(), p);
    }

    /// Whether `p` was changed since the last save.
    pub fn unsaved(&self, p: &Param) -> bool {
        self.current.differs(&self.saved, p)
    }

    pub fn any_unsaved(&self) -> bool {
        PARAMS.iter().any(|p| self.unsaved(p))
    }

    /// Whether settings can be saved on this board.
    pub fn can_save(&self) -> bool {
        self.map.is_some()
    }

    /// Log every setting in use, marking those still at their default.
    pub fn log_all(&self) {
        let defaults = Settings::default();
        for p in &PARAMS {
            let tag = if self.current.differs(&defaults, p) {
                ""
            } else {
                " (default)"
            };
            info!(
                "settings: {:<11} = {}{tag}",
                p.name,
                self.current.display(p)
            );
        }
        if !self.can_save() {
            warn!(
                "settings: no settings partition; built-in defaults only, `param save` unavailable"
            );
        }
        info!("settings: to change them, type `help` on this USB port");
    }
}

/// Find the `nvs` partition and open a map over all of it.
fn open_map(flash: FLASH<'static>) -> Option<Map> {
    static STORAGE: StaticCell<FlashStorage<'static>> = StaticCell::new();
    static TABLE: StaticCell<[u8; PARTITION_TABLE_MAX_LEN]> = StaticCell::new();
    let storage = STORAGE.init(FlashStorage::new(flash));
    let table =
        match partitions::read_partition_table(storage, TABLE.init([0; PARTITION_TABLE_MAX_LEN])) {
            Ok(table) => table,
            Err(e) => {
                warn!("settings: can't read the partition table ({e:?})");
                return None;
            }
        };
    let Ok(Some(nvs)) = table.find_partition(PartitionType::Data(DataPartitionSubType::Nvs)) else {
        warn!("settings: the partition table has no nvs partition");
        return None;
    };
    let region = nvs.as_embedded_storage(storage);
    let size = region.partition_size() as u32;
    match MapConfig::<Flash>::try_new(0..size) {
        Ok(config) => Some(MapStorage::new(
            BlockingAsync::new(region),
            config,
            Cache::new_uncached(),
        )),
        Err(_) => {
            warn!("settings: the nvs partition ({size} bytes) is too small or misaligned");
            None
        }
    }
}
