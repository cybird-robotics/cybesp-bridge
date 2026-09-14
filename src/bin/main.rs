//! cybesp-bridge — Message-agnostic WiFi↔UART bridge for cybflight.
//!
//! Forwards COBS-framed UART data from an STM32H743 flight controller to UDP
//! over WiFi, and vice versa, and feeds RTK corrections to the GNSS receiver.
//! Does not interpret message contents.
//!
//! Runs on several ESP32 boards. A `board-*` Cargo feature selects one, and
//! everything board-specific (which UART and pins carry each link, the status
//! LED, the antenna) lives in `cybesp_bridge::board`; this file only uses the
//! links by role.
//!
//! Scheduling, highest priority first:
//! 1. RTCM corrections → GNSS UART → GNSS receiver (`rtcm_uart_writer_task`,
//!    interrupt executor on SWI1). Every correction counts for RTK.
//! 2. Receive drain of the STM32 link (`fc_rx_drain_task`, interrupt executor
//!    on SWI2). A late drain overruns the 128 B hardware FIFO.
//! 3. Everything network-facing (thread mode). embassy-net's stack is `!Send`,
//!    so socket tasks cannot leave the thread executor; they are decoupled
//!    from the UARTs by static queues instead, and never block them.

#![no_std]
#![no_main]

extern crate alloc;
extern crate esp_backtrace as _;

use alloc::string::ToString;
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, Ordering};
use critical_section::Mutex;
use cybflight_msgs::udp_batch::{self, BatchHeader, BatchWriter};
use cybflight_msgs::wire::FRAME_HEADER_SIZE;
use embassy_executor::Spawner;
use embassy_futures::select::{self, Either};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpAddress, IpEndpoint, Ipv4Address, Runner, Stack, StackResources};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, TrySendError};
use embassy_sync::pipe::Pipe;
use embassy_time::{Duration, Instant, Ticker, Timer};
use esp_hal::interrupt::Priority;
use esp_hal::uart::{self, RxConfig, UartRx, UartTx};
use esp_hal::{Async, Blocking};
use esp_radio::wifi::{
    ClientConfig, ModeConfig, PowerSaveMode, WifiController, WifiDevice, WifiEvent,
};
use esp_rtos::embassy::InterruptExecutor;
use log::{info, warn};
use static_cell::StaticCell;

use cybesp_bridge::board::{self, LinkStatus};
use cybesp_bridge::mk_static;
use cybesp_bridge::settings::{self, Password, Ssid};
use cybesp_bridge::shell;
use esp_hal::peripherals::USB_DEVICE;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// WiFi credentials, the link baud rates and the antenna are settings
// (`cybesp_bridge::settings`), changed from the USB shell: type `help`.
// The ground station's IP is not configured: the bridge learns it from the
// ground station's own traffic (see "Ground station discovery").

/// UDP port the ground station listens on for downlink telemetry.
const DOWNLINK_PORT: u16 = 4210;
/// UDP port this bridge listens on for uplink commands.
const UPLINK_PORT: u16 = 4211;
/// UDP port this bridge listens on for RTCM3 corrections for the GNSS
/// receiver. Datagrams are written verbatim to the GNSS UART — no COBS, no
/// framing. The receiver parses RTCM3 itself (length + CRC24Q).
const RTCM_UPLINK_PORT: u16 = 4212;

// ---------------------------------------------------------------------------
// COBS framing constants (matches cybflight wire format)
// ---------------------------------------------------------------------------

/// Max raw (decoded) frame: [msg_id: u8][seq: u8][payload up to 128 bytes].
const MAX_RAW_FRAME: usize = 2 + 128;
/// Max COBS-encoded frame including 0x00 delimiter.
const MAX_COBS_BUF: usize = cobs::max_encoding_length(MAX_RAW_FRAME) + 1;

// ---------------------------------------------------------------------------
// Link scheduling and buffering (see the module docs for the priority order)
// ---------------------------------------------------------------------------

/// Interrupt priority of the RTCM → GNSS UART writer (SWI1): the highest used.
const RTCM_PRIO: Priority = Priority::Priority3;
/// Interrupt priority of the STM32 link receive drain (SWI2). Both executors sit
/// above the esp-rtos timer / scheduler and the UART driver interrupts
/// (Priority1), so their tasks must stay a few microseconds per poll.
const DRAIN_PRIO: Priority = Priority::Priority2;

/// STM32 link RX FIFO-full wake threshold, in bytes of the 128 B FIFO. The
/// driver default (120) leaves 8 bytes — ~87 µs at 921600 baud — to wake and
/// drain during a burst; 32 leaves 96 bytes, ~1 ms.
const FC_RX_FIFO_THRESHOLD: u16 = 32;

/// Decoded telemetry frames queued between the UART0 drain and the WiFi
/// sender: ~250 ms of full-rate downlink. When full, the OLDEST frame goes.
const DL_QUEUE_LEN: usize = 64;
/// Pack each UART burst into one datagram (`cybflight_msgs::udp_batch`),
/// cutting the radio's packet rate several-fold — airtime the half-duplex
/// radio then has for receiving RTCM. `false` restores one datagram per frame
/// for a ground station that predates the envelope: such a GCS silently
/// discards every batch.
const DOWNLINK_BATCHING: bool = true;
/// Flush backstop when the end-of-burst marker is missed.
const DL_BATCH_IDLE: Duration = Duration::from_micros(1500);
/// Never hold a batch longer than this (the STM32 bursts every 10 ms).
const DL_BATCH_MAX_AGE: Duration = Duration::from_millis(8);
/// Downlink socket TX: a few full batches in flight. smoltcp needs each
/// datagram contiguous and can waste up to one at the ring wrap.
const DL_SOCK_TX_META: usize = 8;
const DL_SOCK_TX_BUF: usize = 4096;

/// RTCM socket RX. An MSM epoch arrives as several datagrams within a few
/// ms, and smoltcp drops silently once slots or bytes run out.
const RTCM_SOCK_RX_META: usize = 16;
const RTCM_SOCK_RX_BUF: usize = 8192;
/// Bytes queued for the GNSS UART: ~710 ms of line time at 115200 baud.
const RTCM_PIPE_SIZE: usize = 8192;

/// WiFi driver packet queues (esp-radio defaults: 5 RX / 3 TX). The RX queue
/// sits before smoltcp, is shared by every inbound packet, and drops silently
/// when full — bigger socket buffers cannot help there.
const WIFI_RX_QUEUE: usize = 12;
const WIFI_TX_QUEUE: usize = 6;

/// Period of the counters log, once turned on from the shell (`stats on`).
const STATS_PERIOD: Duration = Duration::from_secs(5);

/// One decoded wire frame `[msg_id][seq][payload…]`.
#[derive(Clone, Copy)]
struct DlFrame {
    len: u8,
    data: [u8; MAX_RAW_FRAME],
}

impl DlFrame {
    fn bytes(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }
}

enum DlItem {
    Frame(DlFrame),
    /// The UART went idle on a frame boundary: the STM32's burst is complete.
    BurstEnd,
}

/// UART0 drain → downlink sender.
static DL_QUEUE: Channel<CriticalSectionRawMutex, DlItem, DL_QUEUE_LEN> = Channel::new();
/// Set once the downlink socket is up and the ground station is known; the
/// drain discards frames before that rather than counting them as drops.
static DL_ACTIVE: AtomicBool = AtomicBool::new(false);
/// RTCM receiver → GNSS UART writer. Exactly ONE writer (`rtcm_uplink_task`):
/// `rtcm_enqueue`'s capacity check-then-write is race-free only because of it.
static RTCM_PIPE: Pipe<CriticalSectionRawMutex, RTCM_PIPE_SIZE> = Pipe::new();

/// Cumulative counters. Shown by the shell's `stats`, and logged every
/// `STATS_PERIOD` after `stats on`; four of them also ride
/// in every downlink batch header so the ground station sees them.
struct BridgeStats {
    fc_rx_errors: AtomicU32,
    fc_rx_bytes: AtomicU32,
    dl_cobs_errors: AtomicU32,
    dl_queue_drops: AtomicU32,
    dl_queue_hwm: AtomicU32,
    dl_frames_sent: AtomicU32,
    dl_datagrams: AtomicU32,
    dl_send_errors: AtomicU32,
    rtcm_rx_dgrams: AtomicU32,
    rtcm_rx_bytes: AtomicU32,
    rtcm_drops: AtomicU32,
    rtcm_recv_errors: AtomicU32,
    rtcm_pipe_hwm: AtomicU32,
    rtcm_uart_bytes: AtomicU32,
    /// Uplink and RTCM datagrams from a sender other than the ground station.
    uplink_foreign_drops: AtomicU32,
}

impl BridgeStats {
    const fn new() -> Self {
        Self {
            fc_rx_errors: AtomicU32::new(0),
            fc_rx_bytes: AtomicU32::new(0),
            dl_cobs_errors: AtomicU32::new(0),
            dl_queue_drops: AtomicU32::new(0),
            dl_queue_hwm: AtomicU32::new(0),
            dl_frames_sent: AtomicU32::new(0),
            dl_datagrams: AtomicU32::new(0),
            dl_send_errors: AtomicU32::new(0),
            rtcm_rx_dgrams: AtomicU32::new(0),
            rtcm_rx_bytes: AtomicU32::new(0),
            rtcm_drops: AtomicU32::new(0),
            rtcm_recv_errors: AtomicU32::new(0),
            rtcm_pipe_hwm: AtomicU32::new(0),
            rtcm_uart_bytes: AtomicU32::new(0),
            uplink_foreign_drops: AtomicU32::new(0),
        }
    }

    /// The counters the ground station gets in every batch (wrapping u16).
    fn header(&self) -> BatchHeader {
        let g = |c: &AtomicU32| c.load(Ordering::Relaxed) as u16;
        BatchHeader {
            uart_errors: g(&self.fc_rx_errors),
            dl_drops: g(&self.dl_queue_drops).wrapping_add(g(&self.dl_cobs_errors)),
            rtcm_rx: g(&self.rtcm_rx_dgrams),
            rtcm_drops: g(&self.rtcm_drops),
        }
    }
}

static STATS: BridgeStats = BridgeStats::new();

fn inc(c: &AtomicU32) {
    c.fetch_add(1, Ordering::Relaxed);
}

/// Wait until WiFi is associated and DHCP has configured the stack.
async fn wait_for_network(stack: Stack<'static>, who: &str) {
    while !stack.is_link_up() {
        Timer::after_millis(100).await;
    }
    info!("{who}: network link up, waiting for IP...");
    stack.wait_config_up().await;
    info!("{who}: IP acquired, starting");
}

// ---------------------------------------------------------------------------
// Ground station discovery and uplink lock
// ---------------------------------------------------------------------------
//
// The ground station's address is not configured, so any PC running cybgcs
// works without reflashing the bridges. The bridge learns it from what the
// ground station sends to the uplink ports: cybgcs's 10 Hz fleet broadcast
// reaches every drone from startup, and Vicon poses, RTCM corrections and
// ping replies count too. Telemetry and NTP queries go to that address.
//
// The bridge then belongs to that ground station: uplink and RTCM datagrams
// from anyone else are dropped (`uplink_foreign_drops`). The flight
// controller cannot tell two ground stations' streams apart, so a second
// cybgcs started by mistake would otherwise interleave its own fleet state,
// Vicon poses, origin and corrections with the first one's. The rejected
// sender gets a busy notice naming the owner (`udp_batch::busy_notice`, at
// most one per `BUSY_NOTICE_PERIOD`), which cybgcs shows its operator.
//
// Another sender takes over only once the owner has been silent for
// `GS_HANDOVER` (it quit, crashed or lost WiFi). Anything that sends to the
// uplink ports counts as a ground station, including a standalone RTCM source
// or another drone's bridge should one ever send there directly.

/// Silence after which another sender may take over as the ground station.
const GS_HANDOVER: Duration = Duration::from_secs(3);
/// Minimum interval between busy notices, to any rejected sender.
const BUSY_NOTICE_PERIOD: Duration = Duration::from_secs(1);
/// TX buffer of the uplink and RTCM sockets: room for one busy notice.
const NOTICE_TX_BUF: usize = 16;

/// Ground station IPv4 address (`Ipv4Address::to_bits`); 0 until learned.
static GS_IP: AtomicU32 = AtomicU32::new(0);
/// When the ground station was last heard from: ms since boot, wrapping.
static GS_LAST_HEARD_MS: AtomicU32 = AtomicU32::new(0);
/// When the last busy notice went out (ms since boot, wrapping), once one has.
static BUSY_LAST_MS: AtomicU32 = AtomicU32::new(0);
static BUSY_SENT: AtomicBool = AtomicBool::new(false);

fn now_ms() -> u32 {
    Instant::now().as_millis() as u32
}

/// The ground station's address, once it has been heard from.
fn ground_station() -> Option<Ipv4Address> {
    match GS_IP.load(Ordering::Relaxed) {
        0 => None,
        bits => Some(Ipv4Address::from_bits(bits)),
    }
}

/// Whether an uplink datagram from `from` may reach the flight controller:
/// yes if `from` is the ground station, becoming it first if none is known
/// yet or the current one has been silent for `GS_HANDOVER`. Called only
/// from thread-mode tasks, which never preempt one another, so the
/// read-then-write below cannot interleave.
fn accept_uplink(from: IpEndpoint) -> bool {
    #[allow(irrefutable_let_patterns)] // IPv4-only build
    let IpAddress::Ipv4(ip) = from.addr else {
        return false;
    };
    let now = now_ms();
    let current = GS_IP.load(Ordering::Relaxed);
    if ip.to_bits() != current {
        let silent_ms = now.wrapping_sub(GS_LAST_HEARD_MS.load(Ordering::Relaxed));
        if current != 0 && silent_ms < GS_HANDOVER.as_millis() as u32 {
            inc(&STATS.uplink_foreign_drops);
            return false; // the owner is still active
        }
        GS_IP.store(ip.to_bits(), Ordering::Relaxed);
        info!("Ground station: {} (learned from its uplink traffic)", ip);
    }
    GS_LAST_HEARD_MS.store(now, Ordering::Relaxed);
    true
}

/// Tell a rejected sender which ground station owns this bridge, on its
/// downlink port (where cybgcs listens). Rate-limited to one notice per
/// `BUSY_NOTICE_PERIOD`, so the reply traffic stays negligible.
async fn send_busy_notice(socket: &UdpSocket<'_>, to: IpEndpoint) {
    let Some(owner) = ground_station() else { return };
    let now = now_ms();
    if BUSY_SENT.load(Ordering::Relaxed)
        && now.wrapping_sub(BUSY_LAST_MS.load(Ordering::Relaxed))
            < BUSY_NOTICE_PERIOD.as_millis() as u32
    {
        return;
    }
    BUSY_SENT.store(true, Ordering::Relaxed);
    BUSY_LAST_MS.store(now, Ordering::Relaxed);
    warn!("Uplink from {} dropped: this bridge's ground station is {}", to.addr, owner);
    let notice = udp_batch::busy_notice(owner.octets());
    let _ = socket.send_to(&notice, (to.addr, DOWNLINK_PORT)).await;
}

// ---------------------------------------------------------------------------
// LED status — shared state between tasks
// ---------------------------------------------------------------------------

/// Bridge status for the LED, stored as `LinkStatus as u8`.
static BRIDGE_STATUS: AtomicU8 = AtomicU8::new(LinkStatus::Booting as u8);

fn set_status(status: LinkStatus) {
    BRIDGE_STATUS.store(status as u8, Ordering::Relaxed);
}

// ---------------------------------------------------------------------------
// NTP time state — shared between SNTP task and uplink task
// ---------------------------------------------------------------------------

/// NTP sync state: (ntp_time_us, capture_instant_us).
/// `None` means not yet synced.
static NTP_STATE: Mutex<RefCell<Option<(i64, i64)>>> = Mutex::new(RefCell::new(None));

/// Store a new SNTP sync point.
fn set_ntp_sync(ntp_us: i64, instant_us: i64) {
    critical_section::with(|cs| {
        NTP_STATE.borrow_ref_mut(cs).replace((ntp_us, instant_us));
    });
}

/// Get current NTP time estimate by extrapolating from last SNTP sync.
fn get_ntp_time_us() -> Option<i64> {
    critical_section::with(|cs| {
        let state = *NTP_STATE.borrow_ref(cs);
        state.map(|(base, capture)| {
            let now = embassy_time::Instant::now().as_micros() as i64;
            base + (now - capture)
        })
    })
}

/// NTP epoch offset: seconds between 1900-01-01 and 1970-01-01.
const NTP_EPOCH_OFFSET: u64 = 2_208_988_800;

/// Minimal SNTP query: returns server UTC in microseconds since UNIX epoch.
async fn sntp_query(stack: Stack<'static>, server: Ipv4Address) -> Option<i64> {
    let mut rx_meta = [PacketMetadata::EMPTY; 1];
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut rx_buf = [0u8; 64];
    let mut tx_buf = [0u8; 64];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    socket.bind(0).ok()?; // ephemeral port

    // NTP request: LI=0, VN=4, Mode=3 (client)
    let mut request = [0u8; 48];
    request[0] = 0x23;
    socket
        .send_to(&request, (server, 123))
        .await
        .ok()?;

    let mut resp = [0u8; 48];
    let result = embassy_time::with_timeout(
        embassy_time::Duration::from_secs(2),
        socket.recv_from(&mut resp),
    )
    .await;

    let (n, _) = match result {
        Ok(Ok(r)) => r,
        _ => return None,
    };
    if n < 48 {
        return None;
    }

    // Transmit timestamp at offset 40: seconds (u32 BE) + fraction (u32 BE)
    let secs = u32::from_be_bytes([resp[40], resp[41], resp[42], resp[43]]);
    let frac = u32::from_be_bytes([resp[44], resp[45], resp[46], resp[47]]);

    let unix_secs = secs as u64 - NTP_EPOCH_OFFSET;
    let frac_us = (frac as u64 * 1_000_000) >> 32;

    Some((unix_secs * 1_000_000 + frac_us) as i64)
}

// ---------------------------------------------------------------------------
// COBS frame accumulator (ported from cybflight comm/mod.rs)
// ---------------------------------------------------------------------------

/// Result of feeding one byte to [`FrameAccumulator`].
enum Feed {
    /// Mid-frame (or an empty frame between delimiters).
    None,
    /// A complete frame decoded to this many bytes in `data()`.
    Frame(usize),
    /// A frame was discarded: bad COBS, or longer than the buffer.
    Bad,
}

/// Accumulates incoming UART bytes and yields COBS-decoded frames on 0x00 delimiter.
struct FrameAccumulator {
    buf: [u8; MAX_COBS_BUF],
    pos: usize,
    /// Skipping bytes until the next delimiter: after an overflow or a UART
    /// error the bytes that follow are the tail of an unknown frame, and
    /// decoding them as if they were whole could yield a plausible but
    /// corrupt frame (the wire format carries no CRC).
    discarding: bool,
}

impl FrameAccumulator {
    fn new() -> Self {
        Self {
            buf: [0; MAX_COBS_BUF],
            pos: 0,
            discarding: false,
        }
    }

    /// Feed a single byte. On `Feed::Frame(n)` the decoded frame is in
    /// `self.data()[..n]` until the next `feed()` call.
    fn feed(&mut self, byte: u8) -> Feed {
        if byte == 0x00 {
            let pos = self.pos;
            self.pos = 0;
            if core::mem::take(&mut self.discarding) {
                return Feed::Bad;
            }
            if pos == 0 {
                return Feed::None; // empty frame
            }
            match cobs::decode_in_place(&mut self.buf[..pos]) {
                Ok(n) => Feed::Frame(n),
                Err(_) => Feed::Bad,
            }
        } else if self.discarding {
            Feed::None
        } else if self.pos < self.buf.len() {
            self.buf[self.pos] = byte;
            self.pos += 1;
            Feed::None
        } else {
            // Longer than any legal frame — drop it through its delimiter.
            self.pos = 0;
            self.discarding = true;
            Feed::None
        }
    }

    /// Forget any partial frame after a stream error; resume at the next
    /// delimiter.
    fn reset(&mut self) {
        self.pos = 0;
        self.discarding = true;
    }

    /// True between frames (the last byte fed was a delimiter).
    fn at_boundary(&self) -> bool {
        self.pos == 0 && !self.discarding
    }

    /// Access the internal buffer (valid after a `Feed::Frame` return).
    fn data(&self) -> &[u8] {
        &self.buf
    }
}

// ---------------------------------------------------------------------------
// LED status task — each board renders the status its own way
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn led_task(mut led: board::StatusLed) {
    let mut tick: u8 = 0;
    loop {
        led.show(LinkStatus::from_u8(BRIDGE_STATUS.load(Ordering::Relaxed)), tick);
        tick = tick.wrapping_add(1);
        Timer::after_millis(board::LED_TICK_MS).await;
    }
}

// ---------------------------------------------------------------------------
// WiFi connection task — STA mode, auto-reconnect
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn wifi_task(mut controller: WifiController<'static>, ssid: Ssid, password: Password) {
    loop {
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ClientConfig::default()
                .with_ssid(ssid.as_str().to_string())
                .with_password(password.as_str().to_string());
            let config = ModeConfig::Client(client_config);
            controller.set_config(&config).unwrap();
            set_status(LinkStatus::WifiConnecting);
            info!("WiFi: starting STA...");
            controller.start_async().await.unwrap();
        }

        info!("WiFi: connecting to \"{}\"...", ssid);
        match controller.connect_async().await {
            Ok(()) => {
                info!("WiFi: connected");
                set_status(LinkStatus::WifiConnected);
            }
            Err(e) => {
                warn!("WiFi: connect failed: {:?}", e);
                set_status(LinkStatus::WifiConnecting);
                Timer::after_millis(3000).await;
                continue;
            }
        }

        // Stay here while connected; when we drop out, loop reconnects.
        controller.wait_for_event(WifiEvent::StaDisconnected).await;
        warn!("WiFi: disconnected, will reconnect...");
        set_status(LinkStatus::WifiDisconnected);
        Timer::after_millis(1000).await;
    }
}

// ---------------------------------------------------------------------------
// Embassy-net stack runner
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

// ---------------------------------------------------------------------------
// SNTP sync task — periodically syncs to the GS PC NTP server
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn sntp_task(stack: Stack<'static>) {
    // Wait for network.
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after_millis(100).await;
    }
    stack.wait_config_up().await;
    info!("SNTP: network ready, starting sync");

    let mut synced = false;
    let mut warned = false;
    loop {
        // The NTP server is the ground station; nothing to ask until it has
        // been heard from.
        let Some(server) = ground_station() else {
            Timer::after_secs(1).await;
            continue;
        };
        // Log only changes (first sync, lost, back), not every 3 s query:
        // the log shares the port with the settings shell.
        match sntp_query(stack, server).await {
            Some(ntp_us) => {
                let now_us = embassy_time::Instant::now().as_micros() as i64;
                set_ntp_sync(ntp_us, now_us);
                if !synced {
                    info!("SNTP: synced to {}", server);
                    synced = true;
                }
            }
            None if synced || !warned => {
                warn!("SNTP: no reply from {} (is an NTP server running there?)", server);
                synced = false;
                warned = true;
            }
            None => {}
        }
        Timer::after_secs(3).await;
    }
}

// ---------------------------------------------------------------------------
// Downlink stage 1 (interrupt executor, SWI2): STM32 link RX → decoded frames
// ---------------------------------------------------------------------------

/// Takes the Blocking RX half — the Async driver types are `!Send`, and an
/// interrupt executor only accepts `Send` spawns — and converts it here, on
/// the executor that will own it for good.
#[embassy_executor::task]
async fn fc_drain_bootstrap(rx: UartRx<'static, Blocking>) {
    // SAFETY: awaited directly in a task body, so the `Context` is the one
    // the Embassy (interrupt) executor created — the documented requirement.
    // Obtaining an `InterruptExecutor`'s `Spawner` is this API's intended use.
    let spawner = unsafe { Spawner::for_current_executor() }.await;
    spawner.spawn(fc_rx_drain_task(rx.into_async())).unwrap();
}

/// Drains the STM32 link as fast as bytes arrive and queues whole decoded
/// frames. Never touches the network, never sleeps, never logs — so WiFi
/// back-pressure can no longer overrun the UART FIFO.
#[embassy_executor::task]
async fn fc_rx_drain_task(mut rx: UartRx<'static, Async>) {
    let mut acc = FrameAccumulator::new();
    let mut buf = [0u8; 128];
    loop {
        match rx.read_async(&mut buf).await {
            Ok(n) => {
                STATS.fc_rx_bytes.fetch_add(n as u32, Ordering::Relaxed);
                for &b in &buf[..n] {
                    match acc.feed(b) {
                        Feed::None => {}
                        Feed::Frame(len) if (FRAME_HEADER_SIZE..=MAX_RAW_FRAME).contains(&len) => {
                            dl_push_frame(&acc.data()[..len]);
                        }
                        Feed::Frame(_) | Feed::Bad => inc(&STATS.dl_cobs_errors),
                    }
                }
                // A read shorter than the FIFO threshold was woken by the RX
                // idle timeout (~10 symbol times): the STM32's burst is over.
                if n < FC_RX_FIFO_THRESHOLD as usize && acc.at_boundary() {
                    dl_push_burst_end();
                }
            }
            Err(_) => {
                // FIFO overflow / framing / parity. The driver has reset the
                // FIFO; resynchronise on the next delimiter. No sleep: the
                // next burst is already on its way.
                inc(&STATS.fc_rx_errors);
                acc.reset();
                dl_push_burst_end();
            }
        }
    }
}

/// Queue one decoded frame, evicting the oldest when full (fresh telemetry is
/// worth more than old). Runs above the consumer's priority, so the consumer
/// cannot interleave between the eviction and the retry.
fn dl_push_frame(frame: &[u8]) {
    if !DL_ACTIVE.load(Ordering::Relaxed) {
        return;
    }
    let mut f = DlFrame { len: frame.len() as u8, data: [0; MAX_RAW_FRAME] };
    f.data[..frame.len()].copy_from_slice(frame);
    let mut item = DlItem::Frame(f);
    loop {
        match DL_QUEUE.try_send(item) {
            Ok(()) => break,
            Err(TrySendError::Full(back)) => {
                item = back;
                if let Ok(DlItem::Frame(_)) = DL_QUEUE.try_receive() {
                    inc(&STATS.dl_queue_drops);
                }
            }
        }
    }
    STATS.dl_queue_hwm.fetch_max(DL_QUEUE.len() as u32, Ordering::Relaxed);
}

/// Mark the end of a burst. Never evicts a frame to make room for a marker;
/// the sender's idle timer is the backstop.
fn dl_push_burst_end() {
    if DL_ACTIVE.load(Ordering::Relaxed) {
        let _ = DL_QUEUE.try_send(DlItem::BurstEnd);
    }
}

// ---------------------------------------------------------------------------
// Downlink stage 2 (thread mode): decoded frames → UDP to the ground station
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn downlink_task(stack: Stack<'static>) {
    wait_for_network(stack, "Downlink").await;

    let mut rx_meta = [PacketMetadata::EMPTY; 1];
    let mut tx_meta = [PacketMetadata::EMPTY; DL_SOCK_TX_META];
    let mut rx_buf = [0u8; 64];
    let mut tx_buf = [0u8; DL_SOCK_TX_BUF];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    socket.bind(0).unwrap(); // ephemeral local port

    // Telemetry has nowhere to go until the ground station has been heard
    // from; meanwhile the drain discards it (DL_ACTIVE is still false).
    if ground_station().is_none() {
        info!("Downlink: waiting to hear from the ground station (is cybgcs running?)");
        while ground_station().is_none() {
            Timer::after_millis(100).await;
        }
    }

    // Anything queued before now is stale.
    DL_QUEUE.clear();
    DL_ACTIVE.store(true, Ordering::Relaxed);
    info!(
        "Downlink: {} → ground station :{}",
        if DOWNLINK_BATCHING { "batched" } else { "one frame per datagram" },
        DOWNLINK_PORT
    );

    let mut pkt = [0u8; udp_batch::MAX_DATAGRAM];
    loop {
        let first = match DL_QUEUE.receive().await {
            DlItem::Frame(f) => f,
            DlItem::BurstEnd => continue,
        };
        if !DOWNLINK_BATCHING {
            send_datagram(&socket, first.bytes(), 1).await;
            continue;
        }

        // Collect the rest of this burst: until its end marker, an idle gap,
        // or the age cap — whichever comes first.
        let started = Instant::now();
        let mut batch = BatchWriter::new(&mut pkt, STATS.header());
        batch.try_push(first.bytes());
        loop {
            let deadline = (Instant::now() + DL_BATCH_IDLE).min(started + DL_BATCH_MAX_AGE);
            match embassy_time::with_deadline(deadline, DL_QUEUE.receive()).await {
                Ok(DlItem::Frame(f)) => {
                    if !batch.try_push(f.bytes()) {
                        send_datagram(&socket, batch.as_bytes(), batch.frames()).await;
                        batch.reset(STATS.header());
                        batch.try_push(f.bytes());
                    }
                }
                Ok(DlItem::BurstEnd) | Err(_) => break,
            }
        }
        send_datagram(&socket, batch.as_bytes(), batch.frames()).await;
    }
}

/// Send one datagram to the ground station. Errors are counted, never slept on.
async fn send_datagram(socket: &UdpSocket<'_>, bytes: &[u8], frames: usize) {
    // Always known here: the downlink starts only once it has been heard from.
    let Some(gs) = ground_station() else { return };
    match socket.send_to(bytes, (gs, DOWNLINK_PORT)).await {
        Ok(()) => {
            inc(&STATS.dl_datagrams);
            STATS.dl_frames_sent.fetch_add(frames as u32, Ordering::Relaxed);
        }
        Err(_) => inc(&STATS.dl_send_errors),
    }
}

// ---------------------------------------------------------------------------
// Uplink: UDP RX from ground station → COBS encode → UART TX
// ---------------------------------------------------------------------------

/// WireTimeSync msg_id (must match cybflight-msgs/src/wire.rs).
const MSG_ID_TIME_SYNC: u8 = 129;

/// Build a raw WireTimeSync frame: [msg_id, seq, esp_send_ntp_us, prev_tx_complete_ntp_us].
fn build_time_sync_frame(
    esp_send_ntp_us: i64,
    prev_tx_complete_ntp_us: i64,
    seq: &mut u8,
) -> [u8; 18] {
    let mut frame = [0u8; 18]; // 2 header + 16 payload
    frame[0] = MSG_ID_TIME_SYNC;
    frame[1] = *seq;
    *seq = seq.wrapping_add(1);
    frame[2..10].copy_from_slice(&esp_send_ntp_us.to_le_bytes());
    frame[10..18].copy_from_slice(&prev_tx_complete_ntp_us.to_le_bytes());
    frame
}

#[embassy_executor::task]
async fn uplink_task(mut uart_tx: UartTx<'static, esp_hal::Async>, stack: Stack<'static>) {
    // Wait for network to be ready.
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after_millis(100).await;
    }
    info!("Uplink: network link up, waiting for IP...");
    stack.wait_config_up().await;
    info!("Uplink: IP acquired, starting");

    // Uplink carries FLEET_STATE (~128 B @ 10 Hz), GNSS_ORIGIN_SET (1 Hz) and
    // PING_RESP. Sized for a few back-to-back fleet datagrams without
    // silently dropping the small origin/ping frames queued behind them.
    let mut rx_meta = [PacketMetadata::EMPTY; 8];
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut rx_buf = [0u8; 512];
    let mut tx_buf = [0u8; NOTICE_TX_BUF];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    socket.bind(UPLINK_PORT).unwrap();
    info!("Uplink: listening on UDP :{}", UPLINK_PORT);

    let mut recv_buf = [0u8; MAX_RAW_FRAME];
    let mut cobs_buf = [0u8; MAX_COBS_BUF];
    let mut seq: u8 = 0;

    // PTP follow-up state: NTP time at TX-complete of the previous time sync frame.
    let mut prev_tx_complete_ntp_us: i64 = 0;
    let mut time_sync_ticker = Ticker::every(embassy_time::Duration::from_secs(3));

    loop {
        match select::select(
            socket.recv_from(&mut recv_buf),
            time_sync_ticker.next(),
        )
        .await
        {
            Either::First(Ok((n, meta))) => {
                if !accept_uplink(meta.endpoint) {
                    send_busy_notice(&socket, meta.endpoint).await;
                    continue;
                }
                // Normal uplink: COBS-encode the raw datagram and forward to UART.
                let encoded_len = cobs::encode(&recv_buf[..n], &mut cobs_buf);
                cobs_buf[encoded_len] = 0x00;
                let total = encoded_len + 1;

                if let Err(e) =
                    embedded_io_async::Write::write_all(&mut uart_tx, &cobs_buf[..total]).await
                {
                    warn!("Uplink: UART write error: {:?}", e);
                }
            }
            Either::First(Err(e)) => {
                // Only `Truncated` (datagram larger than a frame); no sleep.
                warn!("Uplink: UDP recv error: {:?}", e);
            }
            Either::Second(_) => {
                // Time sync tick: send PTP follow-up frame to STM32.
                if let Some(send_ntp_us) = get_ntp_time_us() {
                    let frame =
                        build_time_sync_frame(send_ntp_us, prev_tx_complete_ntp_us, &mut seq);
                    let encoded_len = cobs::encode(&frame, &mut cobs_buf);
                    cobs_buf[encoded_len] = 0x00;
                    let total = encoded_len + 1;

                    if let Err(e) =
                        embedded_io_async::Write::write_all(&mut uart_tx, &cobs_buf[..total]).await
                    {
                        warn!("Uplink: time sync UART write error: {:?}", e);
                    }

                    // Capture NTP time after write_all() returns for PTP follow-up.
                    // Note: write_all() returns once the last byte is queued in the
                    // 128 B TX FIFO (this UART has no DMA), not at shift-register-
                    // empty, so this timestamp may be up to one frame-duration
                    // (~217 µs at 921.6k baud) early. The resulting constant bias is
                    // absorbed by the STM32's EMA smoother.
                    prev_tx_complete_ntp_us = get_ntp_time_us().unwrap_or(0);
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// RTCM uplink: UDP RX from ground station → raw GNSS UART TX → GNSS receiver
// ---------------------------------------------------------------------------
//
// RTK corrections from cybgcs (NTRIP client or local base) land here. Bytes
// are forwarded verbatim to UART1 — no COBS, no parsing. The receiver parses
// RTCM3 itself (length header + CRC24Q). cybgcs sends one RTCM3 frame per
// datagram, and a datagram is queued whole or not at all, so a drop costs
// exactly one correction message and never a torn one.
//
// The receive loop does nothing but move datagrams into `RTCM_PIPE`, so the
// socket is drained even while the GNSS UART spends ~89 ms shifting out a 1 KB MSM7
// frame at 115200 baud; the UART side runs on the highest-priority executor.

#[embassy_executor::task]
async fn rtcm_uplink_task(stack: Stack<'static>) {
    wait_for_network(stack, "RTCM uplink").await;

    let mut rx_meta = [PacketMetadata::EMPTY; RTCM_SOCK_RX_META];
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut rx_buf = [0u8; RTCM_SOCK_RX_BUF];
    let mut tx_buf = [0u8; NOTICE_TX_BUF];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    socket.bind(RTCM_UPLINK_PORT).unwrap();
    info!("RTCM uplink: listening on UDP :{}", RTCM_UPLINK_PORT);

    let mut recv_buf = [0u8; 1500];
    loop {
        match socket.recv_from(&mut recv_buf).await {
            Ok((n, meta)) => {
                if !accept_uplink(meta.endpoint) {
                    send_busy_notice(&socket, meta.endpoint).await;
                    continue;
                }
                inc(&STATS.rtcm_rx_dgrams);
                STATS.rtcm_rx_bytes.fetch_add(n as u32, Ordering::Relaxed);
                if !rtcm_enqueue(&recv_buf[..n]) {
                    inc(&STATS.rtcm_drops);
                }
            }
            // Only `Truncated` (a datagram over 1500 B is not RTCM). No sleep.
            Err(_) => inc(&STATS.rtcm_recv_errors),
        }
    }
}

/// Queue one whole datagram for UART1, or nothing. `Pipe::try_write` only
/// fills the contiguous part of its ring, hence the loop; with a single
/// writer and capacity checked first it always completes.
fn rtcm_enqueue(d: &[u8]) -> bool {
    if RTCM_PIPE.free_capacity() < d.len() {
        return false;
    }
    let mut off = 0;
    while off < d.len() {
        match RTCM_PIPE.try_write(&d[off..]) {
            Ok(k) => off += k,
            Err(_) => break, // unreachable: see the single-writer note on RTCM_PIPE
        }
    }
    STATS.rtcm_pipe_hwm.fetch_max(RTCM_PIPE.len() as u32, Ordering::Relaxed);
    true
}

/// Takes the Blocking GNSS UART TX half (see `fc_drain_bootstrap`).
#[embassy_executor::task]
async fn rtcm_prio_bootstrap(tx: UartTx<'static, Blocking>) {
    // SAFETY: awaited directly in a task body, so the `Context` is the one
    // the Embassy (interrupt) executor created — the documented requirement.
    // Obtaining an `InterruptExecutor`'s `Spawner` is this API's intended use.
    let spawner = unsafe { Spawner::for_current_executor() }.await;
    spawner.spawn(rtcm_uart_writer_task(tx.into_async())).unwrap();
}

/// Keeps the GNSS UART saturated from `RTCM_PIPE`, on the highest-priority executor,
/// so no WiFi or telemetry work ever delays a correction on its way out.
#[embassy_executor::task]
async fn rtcm_uart_writer_task(mut tx: UartTx<'static, Async>) {
    let mut chunk = [0u8; 256];
    loop {
        let n = RTCM_PIPE.read(&mut chunk).await;
        if embedded_io_async::Write::write_all(&mut tx, &chunk[..n]).await.is_ok() {
            STATS.rtcm_uart_bytes.fetch_add(n as u32, Ordering::Relaxed);
        }
    }
}

// ---------------------------------------------------------------------------
// Settings shell (USB-Serial-JTAG)
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn shell_task(usb: USB_DEVICE<'static>, store: settings::Store) {
    let hooks = shell::Hooks {
        print_stats,
        set_stats_log: |on| STATS_LOG.store(on, Ordering::Relaxed),
    };
    shell::run(usb, store, hooks).await
}

/// The shell's `stats`: link state and every counter, one subsystem per line.
/// The peaks (high-water marks) restart from zero after each call.
fn print_stats() {
    use esp_println::print;
    let g = |c: &AtomicU32| c.load(Ordering::Relaxed);
    let wifi = match LinkStatus::from_u8(BRIDGE_STATUS.load(Ordering::Relaxed)) {
        LinkStatus::Booting => "booting",
        LinkStatus::WifiConnecting => "connecting",
        LinkStatus::WifiConnected => "connected",
        LinkStatus::WifiDisconnected => "disconnected",
    };
    match ground_station() {
        Some(ip) => print!("wifi     {wifi}; ground station {ip}\r\n"),
        None => print!("wifi     {wifi}; no ground station heard yet\r\n"),
    }
    print!(
        "stm32    rx {}B, uart errors {}, bad frames {}\r\n",
        g(&STATS.fc_rx_bytes),
        g(&STATS.fc_rx_errors),
        g(&STATS.dl_cobs_errors),
    );
    print!(
        "downlink frames {}, datagrams {}, queue drops {} (peak {}), send errors {}\r\n",
        g(&STATS.dl_frames_sent),
        g(&STATS.dl_datagrams),
        g(&STATS.dl_queue_drops),
        STATS.dl_queue_hwm.swap(0, Ordering::Relaxed),
        g(&STATS.dl_send_errors),
    );
    print!(
        "rtcm     rx {} ({}B), drops {}, errors {}, buffer peak {}B, to receiver {}B\r\n",
        g(&STATS.rtcm_rx_dgrams),
        g(&STATS.rtcm_rx_bytes),
        g(&STATS.rtcm_drops),
        g(&STATS.rtcm_recv_errors),
        STATS.rtcm_pipe_hwm.swap(0, Ordering::Relaxed),
        g(&STATS.rtcm_uart_bytes),
    );
    print!(
        "uplink   dropped from other ground stations {}\r\n",
        g(&STATS.uplink_foreign_drops),
    );
    print!("heap     {}B free\r\n", esp_alloc::HEAP.free());
}

// ---------------------------------------------------------------------------
// Counters log
// ---------------------------------------------------------------------------

/// Whether `stats_task` logs the counters; toggled from the shell
/// (`stats on|off`). Off at boot, so the console stays quiet while you type.
static STATS_LOG: AtomicBool = AtomicBool::new(false);

#[embassy_executor::task]
async fn stats_task() {
    let mut ticker = Ticker::every(STATS_PERIOD);
    loop {
        ticker.next().await;
        if !STATS_LOG.load(Ordering::Relaxed) {
            continue;
        }
        let g = |c: &AtomicU32| c.load(Ordering::Relaxed);
        info!(
            "stats: fc err {} rx {}B | dl frames {} dgrams {} qdrop {} cobs {} send_err {} qhwm {} \
             | rtcm rx {} ({}B) drop {} err {} pipe_hwm {}B uart {}B | foreign uplink {} \
             | heap free {}B",
            g(&STATS.fc_rx_errors),
            g(&STATS.fc_rx_bytes),
            g(&STATS.dl_frames_sent),
            g(&STATS.dl_datagrams),
            g(&STATS.dl_queue_drops),
            g(&STATS.dl_cobs_errors),
            g(&STATS.dl_send_errors),
            STATS.dl_queue_hwm.swap(0, Ordering::Relaxed),
            g(&STATS.rtcm_rx_dgrams),
            g(&STATS.rtcm_rx_bytes),
            g(&STATS.rtcm_drops),
            g(&STATS.rtcm_recv_errors),
            STATS.rtcm_pipe_hwm.swap(0, Ordering::Relaxed),
            g(&STATS.rtcm_uart_bytes),
            g(&STATS.uplink_foreign_drops),
            esp_alloc::HEAP.free(),
        );
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // 1. Init ESP-HAL with max CPU clock.
    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());
    let peripherals = esp_hal::init(config);

    // 2. Init logging via USB serial.
    // Lines end in CR LF for terminals such as minicom (see the module docs).
    cybesp_bridge::logger::init();

    // 3. Heap allocator — WiFi requires dynamic allocation. Sized for the
    // enlarged WiFi RX queue (each held frame pins a buffer).
    esp_alloc::heap_allocator!(size: 96_000);

    // 4. Settings and board. The settings live in flash, and the board needs
    // some of them (baud rates, antenna) while it claims its UARTs and pins,
    // so it hands the flash peripheral over first. The board then sets up the
    // status LED and antenna (before WiFi init) and returns what the
    // application needs. The UART halves stay Blocking until their tasks
    // convert them on their own executors.
    let mut store = None;
    let board::Board { fc_rx, fc_tx, gnss_tx, led, wifi, timg0, sw_interrupt, usb } =
        board::init(peripherals, |flash| {
            let s = embassy_futures::block_on(settings::Store::open(flash));
            let cfg = board::BoardConfig {
                fc_uart: uart::Config::default()
                    .with_baudrate(s.current.fc_baud)
                    .with_rx(RxConfig::default().with_fifo_full_threshold(FC_RX_FIFO_THRESHOLD)),
                gnss_uart: uart::Config::default().with_baudrate(s.current.gnss_baud),
                ext_antenna: s.current.ext_antenna,
            };
            store = Some(s);
            cfg
        });
    // `init` always calls the closure before returning.
    let store = store.unwrap();
    info!("cybesp-bridge: board {}", board::NAME);
    store.log_all();
    if gnss_tx.is_none() {
        warn!("cybesp-bridge: this board has no GNSS UART; RTCM corrections disabled");
    }
    let settings::Settings { wifi_ssid, wifi_pass, .. } = store.current.clone();

    // 5. Start esp-rtos scheduler (must happen before esp_radio::init).
    let timg0 = esp_hal::timer::timg::TimerGroup::new(timg0);
    let sw_ints = esp_hal::interrupt::software::SoftwareInterruptControl::new(sw_interrupt);
    esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

    spawner.spawn(led_task(led)).unwrap();

    // 6. High-priority executors for the two UART data paths. SWI0 belongs to
    // the esp-rtos scheduler.
    let has_gnss = gnss_tx.is_some();
    if let Some(gnss_tx) = gnss_tx {
        let rtcm_exec = mk_static!(
            InterruptExecutor<1>,
            InterruptExecutor::new(sw_ints.software_interrupt1)
        );
        rtcm_exec.start(RTCM_PRIO).spawn(rtcm_prio_bootstrap(gnss_tx)).unwrap();
    }
    let drain_exec = mk_static!(
        InterruptExecutor<2>,
        InterruptExecutor::new(sw_ints.software_interrupt2)
    );
    drain_exec.start(DRAIN_PRIO).spawn(fc_drain_bootstrap(fc_rx)).unwrap();
    let fc_tx = fc_tx.into_async();

    // 7. Init WiFi radio.
    let radio_init = &*mk_static!(
        esp_radio::Controller<'static>,
        esp_radio::init().unwrap()
    );

    // 8. Create WiFi STA interface.
    // Power save stays off (it is the driver default; explicit here because
    // modem sleep would add beacon-interval latency to every RTCM packet).
    let wifi_config = esp_radio::wifi::Config::default()
        .with_power_save_mode(PowerSaveMode::None)
        .with_rx_queue_size(WIFI_RX_QUEUE)
        .with_tx_queue_size(WIFI_TX_QUEUE);
    let (controller, interfaces) =
        esp_radio::wifi::new(radio_init, wifi, wifi_config).unwrap();
    let sta_interface = interfaces.sta;

    // 9. Configure embassy-net with DHCP.
    let seed = {
        let rng = esp_hal::rng::Rng::new();
        (rng.random() as u64) | ((rng.random() as u64) << 32)
    };

    // 4 user sockets (downlink, uplink, rtcm_uplink, sntp) + DHCP + DNS.
    static RESOURCES: StaticCell<StackResources<8>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        sta_interface,
        embassy_net::Config::dhcpv4(Default::default()),
        RESOURCES.init(StackResources::new()),
        seed,
    );

    // 10. Spawn tasks.
    spawner.spawn(wifi_task(controller, wifi_ssid, wifi_pass)).unwrap();
    spawner.spawn(net_task(runner)).unwrap();
    if has_gnss {
        spawner.spawn(rtcm_uplink_task(stack)).unwrap();
    }
    spawner.spawn(downlink_task(stack)).unwrap();
    spawner.spawn(uplink_task(fc_tx, stack)).unwrap();
    spawner.spawn(sntp_task(stack)).unwrap();
    spawner.spawn(stats_task()).unwrap();
    spawner.spawn(shell_task(usb, store)).unwrap();

    info!("cybesp-bridge: all tasks spawned");

    // Main task idles — all work happens in spawned tasks.
    loop {
        Timer::after_secs(60).await;
    }
}
