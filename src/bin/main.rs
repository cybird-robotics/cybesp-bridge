//! cybesp-bridge — Message-agnostic WiFi↔UART bridge for cybflight.
//!
//! Runs on either the Waveshare ESP32-C6-Zero or the Seeed XIAO ESP32-C6,
//! selected at build time via the `board-waveshare` (default) or
//! `board-xiao-c6` Cargo feature. Forwards COBS-framed UART data from an
//! STM32H743 flight controller to UDP over WiFi, and vice versa, and feeds
//! RTK corrections to the GNSS receiver. Does not interpret message contents.
//!
//! Scheduling, highest priority first:
//! 1. RTCM corrections → UART1 → GNSS receiver (`rtcm_uart_writer_task`,
//!    interrupt executor on SWI1). Every correction counts for RTK.
//! 2. UART0 receive drain from the STM32 (`uart0_rx_drain_task`, interrupt
//!    executor on SWI2). A late drain overruns the 128 B hardware FIFO.
//! 3. Everything network-facing (thread mode). embassy-net's stack is `!Send`,
//!    so socket tasks cannot leave the thread executor; they are decoupled
//!    from the UARTs by static queues instead, and never block them.

#![no_std]
#![no_main]

#[cfg(all(feature = "board-waveshare", feature = "board-xiao-c6"))]
compile_error!("select exactly one board feature: board-waveshare OR board-xiao-c6");
#[cfg(not(any(feature = "board-waveshare", feature = "board-xiao-c6")))]
compile_error!("select a board feature: board-waveshare or board-xiao-c6");

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
use embassy_net::{Ipv4Address, Runner, Stack, StackResources};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, TrySendError};
use embassy_sync::pipe::Pipe;
use embassy_time::{Duration, Instant, Ticker, Timer};
#[cfg(feature = "board-xiao-c6")]
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::interrupt::Priority;
#[cfg(feature = "board-waveshare")]
use esp_hal::rmt::{PulseCode, Rmt};
use esp_hal::uart::{RxConfig, UartRx, UartTx};
use esp_hal::{Async, Blocking};
#[cfg(feature = "board-waveshare")]
use esp_hal_smartled::SmartLedsAdapter;
use esp_radio::wifi::{
    ClientConfig, ModeConfig, PowerSaveMode, WifiController, WifiDevice, WifiEvent,
};
use esp_rtos::embassy::InterruptExecutor;
use log::{info, warn};
#[cfg(feature = "board-waveshare")]
use smart_leds::SmartLedsWrite;
use static_cell::StaticCell;

use cybesp_bridge::mk_static;

/// RMT buffer size for 1 WS2812 LED (24 bits + 1 reset pulse).
#[cfg(feature = "board-waveshare")]
const LED_RMT_BUFFER_SIZE: usize = 25;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// WiFi network SSID to join.
const WIFI_SSID: &str = "UTADR";
/// WiFi password.
const WIFI_PASSWORD: &str = "adr2025WIN";
/// Ground station IP address on the network.
// const GROUND_STATION_IP: Ipv4Address = Ipv4Address::new(192, 168, 50, 101);
const GROUND_STATION_IP: Ipv4Address = Ipv4Address::new(192, 168, 50, 224); // WIFI

/// UDP port the ground station listens on for downlink telemetry.
const DOWNLINK_PORT: u16 = 4210;
/// UDP port this bridge listens on for uplink commands.
const UPLINK_PORT: u16 = 4211;
/// UDP port this bridge listens on for RTCM3 corrections destined for
/// the ublox GPS UART2. Datagrams are written verbatim to UART1 — no
/// COBS, no framing. The ublox parses RTCM3 itself (length + CRC24Q).
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

/// Interrupt priority of the RTCM → UART1 writer (SWI1): the highest used.
const RTCM_PRIO: Priority = Priority::Priority3;
/// Interrupt priority of the UART0 receive drain (SWI2). Both executors sit
/// above the esp-rtos timer / scheduler and the UART driver interrupts
/// (Priority1), so their tasks must stay a few microseconds per poll.
const DRAIN_PRIO: Priority = Priority::Priority2;

/// UART0 RX FIFO-full wake threshold, in bytes of the 128 B FIFO. The driver
/// default (120) leaves 8 bytes — ~87 µs at 921600 baud — to wake and drain
/// during a burst; 32 leaves 96 bytes, ~1 ms.
const UART0_RX_FIFO_THRESHOLD: u16 = 32;

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
/// Bytes queued for UART1: ~710 ms of line time at 115200 baud.
const RTCM_PIPE_SIZE: usize = 8192;

/// WiFi driver packet queues (esp-radio defaults: 5 RX / 3 TX). The RX queue
/// sits before smoltcp, is shared by every inbound packet, and drops silently
/// when full — bigger socket buffers cannot help there.
const WIFI_RX_QUEUE: usize = 12;
const WIFI_TX_QUEUE: usize = 6;

/// Period of the counters log line.
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
/// Set once the downlink socket is up; the drain discards frames before that
/// rather than counting boot-time frames as drops.
static DL_ACTIVE: AtomicBool = AtomicBool::new(false);
/// RTCM receiver → UART1 writer. Exactly ONE writer (`rtcm_uplink_task`):
/// `rtcm_enqueue`'s capacity check-then-write is race-free only because of it.
static RTCM_PIPE: Pipe<CriticalSectionRawMutex, RTCM_PIPE_SIZE> = Pipe::new();

/// Cumulative counters. Logged every `STATS_PERIOD`; four of them also ride
/// in every downlink batch header so the ground station sees them.
struct BridgeStats {
    uart0_rx_errors: AtomicU32,
    uart0_rx_bytes: AtomicU32,
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
}

impl BridgeStats {
    const fn new() -> Self {
        Self {
            uart0_rx_errors: AtomicU32::new(0),
            uart0_rx_bytes: AtomicU32::new(0),
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
        }
    }

    /// The counters the ground station gets in every batch (wrapping u16).
    fn header(&self) -> BatchHeader {
        let g = |c: &AtomicU32| c.load(Ordering::Relaxed) as u16;
        BatchHeader {
            uart_errors: g(&self.uart0_rx_errors),
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
// LED status — shared state between tasks
// ---------------------------------------------------------------------------

/// Bridge status for LED indicator.
const STATUS_BOOTING: u8 = 0;
const STATUS_WIFI_CONNECTING: u8 = 1;
const STATUS_WIFI_CONNECTED: u8 = 2;
const STATUS_WIFI_DISCONNECTED: u8 = 3;

static BRIDGE_STATUS: AtomicU8 = AtomicU8::new(STATUS_BOOTING);

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
async fn sntp_query(stack: Stack<'static>) -> Option<i64> {
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
        .send_to(&request, (GROUND_STATION_IP, 123))
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
// LED status task — WS2812 on GPIO8
// ---------------------------------------------------------------------------

/// LED patterns (Waveshare WS2812 RGB):
///   Booting:           breathing blue
///   WiFi connecting:   breathing blue
///   WiFi connected:    solid green (dim)
///   WiFi disconnected: blinking red
#[cfg(feature = "board-waveshare")]
#[embassy_executor::task]
async fn led_task(mut led: SmartLedsAdapter<'static, LED_RMT_BUFFER_SIZE>) {
    let mut tick: u8 = 0;
    loop {
        let status = BRIDGE_STATUS.load(Ordering::Relaxed);
        // ESP32-C6-Zero onboard LED uses RGB wire order (not GRB).
        // SmartLedsAdapter sends Grb fields [g, r, b] in memory order,
        // so we put our intended R into the g field, G into r field.
        let color = match status {
            STATUS_BOOTING => {
                // Pulsing blue
                let v = triangle_wave(tick, 40);
                rgb::Grb { g: 0, r: 0, b: v }
            }
            STATUS_WIFI_CONNECTING => {
                // Breathing blue
                let v = triangle_wave(tick, 30);
                rgb::Grb { g: 0, r: 0, b: v }
            }
            STATUS_WIFI_CONNECTED => {
                // Solid green, dim — put green value in r field (2nd byte = G on wire)
                rgb::Grb { g: 0, r: 15, b: 0 }
            }
            STATUS_WIFI_DISCONNECTED => {
                // Blinking red — put red value in g field (1st byte = R on wire)
                let on = (tick / 5) % 2 == 0;
                if on { rgb::Grb { g: 40, r: 0, b: 0 } } else { rgb::Grb { g: 0, r: 0, b: 0 } }
            }
            _ => rgb::Grb { g: 0, r: 0, b: 0 },
        };

        let _ = led.write([color].into_iter());
        tick = tick.wrapping_add(1);
        Timer::after_millis(50).await;
    }
}

/// Triangle wave: ramps 0→max→0 over `period` ticks.
#[cfg(feature = "board-waveshare")]
fn triangle_wave(tick: u8, max_brightness: u8) -> u8 {
    // period = 2 * 50 ticks = 100 ticks = 5 seconds full cycle at 50ms
    let phase = tick % 100;
    let ramp = if phase < 50 { phase } else { 100 - phase };
    ((ramp as u16 * max_brightness as u16) / 50) as u8
}

/// Logical "on" level for the XIAO ESP32-C6 user LED (GPIO15). The onboard
/// LED is active-low (driving the pin LOW lights it). Flip this single const
/// if a board revision wires it active-high.
#[cfg(feature = "board-xiao-c6")]
const LED_ON: Level = Level::Low;
#[cfg(feature = "board-xiao-c6")]
const LED_OFF: Level = Level::High;

/// LED patterns (XIAO single monochrome LED on GPIO15 — status conveyed via
/// blink rate since there is no color):
///   Booting / WiFi connecting: slow breathe-style blink (~1 Hz)
///   WiFi connected:            solid on
///   WiFi disconnected:         fast blink (~5 Hz)
#[cfg(feature = "board-xiao-c6")]
#[embassy_executor::task]
async fn led_task(mut led: Output<'static>) {
    let mut tick: u8 = 0;
    loop {
        let status = BRIDGE_STATUS.load(Ordering::Relaxed);
        let on = match status {
            // Slow blink: on for half of a ~1s (20-tick) period.
            STATUS_BOOTING | STATUS_WIFI_CONNECTING => (tick / 10) % 2 == 0,
            // Solid on while linked.
            STATUS_WIFI_CONNECTED => true,
            // Fast blink (~5 Hz): toggle every 2 ticks (100ms).
            STATUS_WIFI_DISCONNECTED => (tick / 2) % 2 == 0,
            _ => false,
        };
        led.set_level(if on { LED_ON } else { LED_OFF });
        tick = tick.wrapping_add(1);
        Timer::after_millis(50).await;
    }
}

// ---------------------------------------------------------------------------
// WiFi connection task — STA mode, auto-reconnect
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn wifi_task(mut controller: WifiController<'static>) {
    loop {
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = ClientConfig::default()
                .with_ssid(WIFI_SSID.to_string())
                .with_password(WIFI_PASSWORD.to_string());
            let config = ModeConfig::Client(client_config);
            controller.set_config(&config).unwrap();
            BRIDGE_STATUS.store(STATUS_WIFI_CONNECTING, Ordering::Relaxed);
            info!("WiFi: starting STA...");
            controller.start_async().await.unwrap();
        }

        info!("WiFi: connecting to \"{}\"...", WIFI_SSID);
        match controller.connect_async().await {
            Ok(()) => {
                info!("WiFi: connected");
                BRIDGE_STATUS.store(STATUS_WIFI_CONNECTED, Ordering::Relaxed);
            }
            Err(e) => {
                warn!("WiFi: connect failed: {:?}", e);
                BRIDGE_STATUS.store(STATUS_WIFI_CONNECTING, Ordering::Relaxed);
                Timer::after_millis(3000).await;
                continue;
            }
        }

        // Stay here while connected; when we drop out, loop reconnects.
        controller.wait_for_event(WifiEvent::StaDisconnected).await;
        warn!("WiFi: disconnected, will reconnect...");
        BRIDGE_STATUS.store(STATUS_WIFI_DISCONNECTED, Ordering::Relaxed);
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

    loop {
        match sntp_query(stack).await {
            Some(ntp_us) => {
                let now_us = embassy_time::Instant::now().as_micros() as i64;
                set_ntp_sync(ntp_us, now_us);
                info!("SNTP: synced (offset captured)");
            }
            None => {
                warn!("SNTP: query failed");
            }
        }
        Timer::after_secs(3).await;
    }
}

// ---------------------------------------------------------------------------
// Downlink stage 1 (interrupt executor, SWI2): UART0 RX → decoded frames
// ---------------------------------------------------------------------------

/// Takes the Blocking RX half — the Async driver types are `!Send`, and an
/// interrupt executor only accepts `Send` spawns — and converts it here, on
/// the executor that will own it for good.
#[embassy_executor::task]
async fn drain_prio_bootstrap(rx: UartRx<'static, Blocking>) {
    // SAFETY: awaited directly in a task body, so the `Context` is the one
    // the Embassy (interrupt) executor created — the documented requirement.
    // Obtaining an `InterruptExecutor`'s `Spawner` is this API's intended use.
    let spawner = unsafe { Spawner::for_current_executor() }.await;
    spawner.spawn(uart0_rx_drain_task(rx.into_async())).unwrap();
}

/// Drains the STM32 link as fast as bytes arrive and queues whole decoded
/// frames. Never touches the network, never sleeps, never logs — so WiFi
/// back-pressure can no longer overrun the UART FIFO.
#[embassy_executor::task]
async fn uart0_rx_drain_task(mut rx: UartRx<'static, Async>) {
    let mut acc = FrameAccumulator::new();
    let mut buf = [0u8; 128];
    loop {
        match rx.read_async(&mut buf).await {
            Ok(n) => {
                STATS.uart0_rx_bytes.fetch_add(n as u32, Ordering::Relaxed);
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
                if n < UART0_RX_FIFO_THRESHOLD as usize && acc.at_boundary() {
                    dl_push_burst_end();
                }
            }
            Err(_) => {
                // FIFO overflow / framing / parity. The driver has reset the
                // FIFO; resynchronise on the next delimiter. No sleep: the
                // next burst is already on its way.
                inc(&STATS.uart0_rx_errors);
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

    // Anything queued before the network came up is stale.
    DL_QUEUE.clear();
    DL_ACTIVE.store(true, Ordering::Relaxed);
    info!(
        "Downlink: {} → {}:{}",
        if DOWNLINK_BATCHING { "batched" } else { "one frame per datagram" },
        GROUND_STATION_IP,
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
    match socket.send_to(bytes, (GROUND_STATION_IP, DOWNLINK_PORT)).await {
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
    let mut tx_buf = [0u8; 1];
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
            Either::First(Ok((n, _remote))) => {
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
// RTCM uplink: UDP RX from ground station → raw UART1 TX → GNSS receiver
// ---------------------------------------------------------------------------
//
// RTK corrections from cybgcs (NTRIP client or local base) land here. Bytes
// are forwarded verbatim to UART1 — no COBS, no parsing. The receiver parses
// RTCM3 itself (length header + CRC24Q). cybgcs sends one RTCM3 frame per
// datagram, and a datagram is queued whole or not at all, so a drop costs
// exactly one correction message and never a torn one.
//
// The receive loop does nothing but move datagrams into `RTCM_PIPE`, so the
// socket is drained even while UART1 spends ~89 ms shifting out a 1 KB MSM7
// frame at 115200 baud; the UART side runs on the highest-priority executor.

#[embassy_executor::task]
async fn rtcm_uplink_task(stack: Stack<'static>) {
    wait_for_network(stack, "RTCM uplink").await;

    let mut rx_meta = [PacketMetadata::EMPTY; RTCM_SOCK_RX_META];
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut rx_buf = [0u8; RTCM_SOCK_RX_BUF];
    let mut tx_buf = [0u8; 1];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    socket.bind(RTCM_UPLINK_PORT).unwrap();
    info!("RTCM uplink: listening on UDP :{}", RTCM_UPLINK_PORT);

    let mut recv_buf = [0u8; 1500];
    loop {
        match socket.recv_from(&mut recv_buf).await {
            Ok((n, _)) => {
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

/// Takes the Blocking UART1 TX half (see `drain_prio_bootstrap`).
#[embassy_executor::task]
async fn rtcm_prio_bootstrap(tx: UartTx<'static, Blocking>) {
    // SAFETY: awaited directly in a task body, so the `Context` is the one
    // the Embassy (interrupt) executor created — the documented requirement.
    // Obtaining an `InterruptExecutor`'s `Spawner` is this API's intended use.
    let spawner = unsafe { Spawner::for_current_executor() }.await;
    spawner.spawn(rtcm_uart_writer_task(tx.into_async())).unwrap();
}

/// Keeps UART1 saturated from `RTCM_PIPE`, on the highest-priority executor,
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
// Counters log
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn stats_task() {
    let mut ticker = Ticker::every(STATS_PERIOD);
    loop {
        ticker.next().await;
        let g = |c: &AtomicU32| c.load(Ordering::Relaxed);
        info!(
            "stats: uart0 err {} rx {}B | dl frames {} dgrams {} qdrop {} cobs {} send_err {} qhwm {} \
             | rtcm rx {} ({}B) drop {} err {} pipe_hwm {}B uart {}B | heap free {}B",
            g(&STATS.uart0_rx_errors),
            g(&STATS.uart0_rx_bytes),
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
    esp_println::logger::init_logger_from_env();

    // 3. Heap allocator — WiFi requires dynamic allocation. Sized for the
    // enlarged WiFi RX queue (each held frame pins a buffer).
    esp_alloc::heap_allocator!(size: 96_000);

    // 4. Start esp-rtos scheduler (must happen before esp_radio::init).
    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let sw_ints =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

    // 5. Init status LED.
    #[cfg(feature = "board-waveshare")]
    {
        // WS2812 RGB LED on GPIO8 via RMT channel 0.
        let rmt = Rmt::new(peripherals.RMT, esp_hal::time::Rate::from_mhz(80)).unwrap();
        static LED_BUF: StaticCell<[PulseCode; LED_RMT_BUFFER_SIZE]> = StaticCell::new();
        let rmt_buf = LED_BUF.init([PulseCode::default(); LED_RMT_BUFFER_SIZE]);
        let led = SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO8, rmt_buf);
        spawner.spawn(led_task(led)).unwrap();
    }
    #[cfg(feature = "board-xiao-c6")]
    {
        // Single monochrome user LED on GPIO15.
        let led = Output::new(peripherals.GPIO15, LED_OFF, OutputConfig::default());
        spawner.spawn(led_task(led)).unwrap();
    }

    // 6. Init UART0 for STM32 communication (921600 baud, 8N1). The RX half
    // stays Blocking until the drain task converts it on its own executor.
    let uart_config = esp_hal::uart::Config::default()
        .with_baudrate(921_600)
        .with_rx(RxConfig::default().with_fifo_full_threshold(UART0_RX_FIFO_THRESHOLD));
    let (uart_rx, uart_tx) = esp_hal::uart::Uart::new(peripherals.UART0, uart_config)
        .unwrap()
        .with_tx(peripherals.GPIO16)
        .with_rx(peripherals.GPIO17)
        .split();
    let uart_tx = uart_tx.into_async();

    info!("cybesp-bridge: UART0 init OK (921600 baud)");

    // 6a. Init UART1 for RTCM3 forwarding to ublox UART2 (115200 baud, TX-only).
    // The ublox doesn't talk back on UART2, so we don't bind an RX pin.
    // Pin: GPIO14 on Waveshare; GPIO0 on XIAO (GPIO14 is the antenna switch there).
    let rtcm_uart_config = esp_hal::uart::Config::default().with_baudrate(115_200);
    let rtcm_uart_builder = esp_hal::uart::Uart::new(peripherals.UART1, rtcm_uart_config).unwrap();
    #[cfg(feature = "board-waveshare")]
    let rtcm_uart = rtcm_uart_builder.with_tx(peripherals.GPIO14);
    #[cfg(feature = "board-xiao-c6")]
    let rtcm_uart = rtcm_uart_builder.with_tx(peripherals.GPIO0);
    let (_, rtcm_uart_tx) = rtcm_uart.split(); // Blocking; converted by its task

    info!("cybesp-bridge: UART1 init OK (115200 baud, RTCM3 → GNSS receiver)");

    // 6b. High-priority executors for the two UART data paths. SWI0 belongs to
    // the esp-rtos scheduler.
    let rtcm_exec = mk_static!(
        InterruptExecutor<1>,
        InterruptExecutor::new(sw_ints.software_interrupt1)
    );
    rtcm_exec.start(RTCM_PRIO).spawn(rtcm_prio_bootstrap(rtcm_uart_tx)).unwrap();
    let drain_exec = mk_static!(
        InterruptExecutor<2>,
        InterruptExecutor::new(sw_ints.software_interrupt2)
    );
    drain_exec.start(DRAIN_PRIO).spawn(drain_prio_bootstrap(uart_rx)).unwrap();

    // 6c. XIAO ESP32-C6 antenna switch: enable the RF switch (GPIO3 LOW) and
    // select the built-in PCB antenna (GPIO14 LOW). Named bindings keep the
    // pins driven for the program lifetime (main never returns).
    #[cfg(feature = "board-xiao-c6")]
    let _rf_enable = Output::new(peripherals.GPIO3, Level::Low, OutputConfig::default());
    #[cfg(feature = "board-xiao-c6")]
    let _ant_select = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default());

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
        esp_radio::wifi::new(radio_init, peripherals.WIFI, wifi_config).unwrap();
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
    spawner.spawn(wifi_task(controller)).unwrap();
    spawner.spawn(net_task(runner)).unwrap();
    spawner.spawn(rtcm_uplink_task(stack)).unwrap();
    spawner.spawn(downlink_task(stack)).unwrap();
    spawner.spawn(uplink_task(uart_tx, stack)).unwrap();
    spawner.spawn(sntp_task(stack)).unwrap();
    spawner.spawn(stats_task()).unwrap();

    info!("cybesp-bridge: all tasks spawned");

    // Main task idles — all work happens in spawned tasks.
    loop {
        Timer::after_secs(60).await;
    }
}
