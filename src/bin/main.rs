//! cybesp-bridge — Message-agnostic WiFi↔UART bridge for cybflight.
//!
//! Runs on Waveshare ESP32-C6-Zero. Forwards COBS-framed UART data from
//! an STM32H743 flight controller to raw UDP datagrams over WiFi, and
//! vice versa. Does not interpret message contents.

#![no_std]
#![no_main]

extern crate alloc;
extern crate esp_backtrace as _;

use alloc::string::ToString;
use core::sync::atomic::{AtomicU8, Ordering};
use embassy_executor::Spawner;
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{Ipv4Address, Runner, Stack, StackResources};
use embassy_time::Timer;
use esp_hal::rmt::{PulseCode, Rmt};
use esp_hal::uart::{UartRx, UartTx};
use esp_hal_smartled::SmartLedsAdapter;
use esp_radio::wifi::{ClientConfig, ModeConfig, WifiController, WifiDevice, WifiEvent};
use log::{info, warn};
use smart_leds::SmartLedsWrite;
use static_cell::StaticCell;

use cybesp_bridge::mk_static;

/// RMT buffer size for 1 WS2812 LED (24 bits + 1 reset pulse).
const LED_RMT_BUFFER_SIZE: usize = 25;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// WiFi network SSID to join.
const WIFI_SSID: &str = "UTADR";
/// WiFi password.
const WIFI_PASSWORD: &str = "adr2025WIN";
/// Ground station IP address on the network.
const GROUND_STATION_IP: Ipv4Address = Ipv4Address::new(192, 168, 50, 101);
/// UDP port the ground station listens on for downlink telemetry.
const DOWNLINK_PORT: u16 = 4210;
/// UDP port this bridge listens on for uplink commands.
const UPLINK_PORT: u16 = 4211;

// ---------------------------------------------------------------------------
// COBS framing constants (matches cybflight wire format)
// ---------------------------------------------------------------------------

/// Max raw (decoded) frame: [msg_id: u8][seq: u8][payload up to 64 bytes].
const MAX_RAW_FRAME: usize = 2 + 64;
/// Max COBS-encoded frame including 0x00 delimiter.
const MAX_COBS_BUF: usize = cobs::max_encoding_length(MAX_RAW_FRAME) + 1;

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
// COBS frame accumulator (ported from cybflight comm/mod.rs)
// ---------------------------------------------------------------------------

/// Accumulates incoming UART bytes and yields COBS-decoded frames on 0x00 delimiter.
struct FrameAccumulator {
    buf: [u8; MAX_COBS_BUF],
    pos: usize,
}

impl FrameAccumulator {
    fn new() -> Self {
        Self {
            buf: [0; MAX_COBS_BUF],
            pos: 0,
        }
    }

    /// Feed a single byte. Returns `Some(decoded_len)` when a complete frame
    /// is received and successfully COBS-decoded. The decoded data is in
    /// `self.data()[..decoded_len]` until the next `feed()` call.
    fn feed(&mut self, byte: u8) -> Option<usize> {
        if byte == 0x00 {
            if self.pos == 0 {
                return None; // empty frame
            }
            let pos = self.pos;
            self.pos = 0;
            cobs::decode_in_place(&mut self.buf[..pos]).ok()
        } else if self.pos < self.buf.len() {
            self.buf[self.pos] = byte;
            self.pos += 1;
            None
        } else {
            // Buffer overflow — discard frame.
            self.pos = 0;
            None
        }
    }

    /// Access the internal buffer (valid after a successful `feed()` return).
    fn data(&self) -> &[u8] {
        &self.buf
    }
}

// ---------------------------------------------------------------------------
// LED status task — WS2812 on GPIO8
// ---------------------------------------------------------------------------

/// LED patterns:
///   Booting:           breathing blue
///   WiFi connecting:   breathing blue
///   WiFi connected:    solid green (dim)
///   WiFi disconnected: blinking red
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
fn triangle_wave(tick: u8, max_brightness: u8) -> u8 {
    // period = 2 * 50 ticks = 100 ticks = 5 seconds full cycle at 50ms
    let phase = tick % 100;
    let ramp = if phase < 50 { phase } else { 100 - phase };
    ((ramp as u16 * max_brightness as u16) / 50) as u8
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
// Downlink: UART RX → COBS decode → UDP TX to ground station
// ---------------------------------------------------------------------------

#[embassy_executor::task]
async fn downlink_task(mut uart_rx: UartRx<'static, esp_hal::Async>, stack: Stack<'static>) {
    // Wait for network to be ready.
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after_millis(100).await;
    }
    info!("Downlink: network link up, waiting for IP...");
    stack.wait_config_up().await;
    info!("Downlink: IP acquired, starting");

    let mut rx_meta = [PacketMetadata::EMPTY; 2];
    let mut tx_meta = [PacketMetadata::EMPTY; 2];
    let mut rx_buf = [0u8; 128];
    let mut tx_buf = [0u8; 128];
    let socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);

    let remote = (GROUND_STATION_IP, DOWNLINK_PORT);

    let mut acc = FrameAccumulator::new();
    let mut uart_buf = [0u8; 256];

    loop {
        match embedded_io_async::Read::read(&mut uart_rx, &mut uart_buf).await {
            Ok(n) if n > 0 => {
                for &b in &uart_buf[..n] {
                    if let Some(len) = acc.feed(b) {
                        // Complete decoded frame — send as UDP datagram.
                        let data = &acc.data()[..len];
                        if let Err(e) = socket.send_to(data, remote).await {
                            warn!("Downlink: UDP send error: {:?}", e);
                        }
                    }
                }
            }
            Ok(_) => {}
            Err(_) => {
                warn!("Downlink: UART read error");
                Timer::after_millis(10).await;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Uplink: UDP RX from ground station → COBS encode → UART TX
// ---------------------------------------------------------------------------

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

    let mut rx_meta = [PacketMetadata::EMPTY; 4];
    let mut tx_meta = [PacketMetadata::EMPTY; 1];
    let mut rx_buf = [0u8; 256];
    let mut tx_buf = [0u8; 1];
    let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    socket.bind(UPLINK_PORT).unwrap();
    info!("Uplink: listening on UDP :{}", UPLINK_PORT);

    let mut recv_buf = [0u8; MAX_RAW_FRAME];
    let mut cobs_buf = [0u8; MAX_COBS_BUF];

    loop {
        match socket.recv_from(&mut recv_buf).await {
            Ok((n, _remote)) => {
                // COBS-encode the raw datagram and append 0x00 delimiter.
                let encoded_len = cobs::encode(&recv_buf[..n], &mut cobs_buf);
                cobs_buf[encoded_len] = 0x00;
                let total = encoded_len + 1;

                if let Err(e) =
                    embedded_io_async::Write::write_all(&mut uart_tx, &cobs_buf[..total]).await
                {
                    warn!("Uplink: UART write error: {:?}", e);
                }
            }
            Err(e) => {
                warn!("Uplink: UDP recv error: {:?}", e);
                Timer::after_millis(10).await;
            }
        }
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

    // 3. Heap allocator — WiFi requires dynamic allocation.
    esp_alloc::heap_allocator!(size: 72_000);

    // 4. Start esp-rtos scheduler (must happen before esp_radio::init).
    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let sw_ints =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

    // 5. Init WS2812 LED on GPIO8 via RMT channel 0.
    let rmt = Rmt::new(peripherals.RMT, esp_hal::time::Rate::from_mhz(80)).unwrap();
    static LED_BUF: StaticCell<[PulseCode; LED_RMT_BUFFER_SIZE]> = StaticCell::new();
    let rmt_buf = LED_BUF.init([PulseCode::default(); LED_RMT_BUFFER_SIZE]);
    let led = SmartLedsAdapter::new(rmt.channel0, peripherals.GPIO8, rmt_buf);
    spawner.spawn(led_task(led)).unwrap();

    // 6. Init UART0 for STM32 communication (921600 baud, 8N1).
    let uart_config = esp_hal::uart::Config::default().with_baudrate(921_600);
    let uart = esp_hal::uart::Uart::new(peripherals.UART0, uart_config)
        .unwrap()
        .with_tx(peripherals.GPIO16)
        .with_rx(peripherals.GPIO17)
        .into_async();
    let (uart_rx, uart_tx) = uart.split();

    info!("cybesp-bridge: UART0 init OK (921600 baud)");

    // 7. Init WiFi radio.
    let radio_init = &*mk_static!(
        esp_radio::Controller<'static>,
        esp_radio::init().unwrap()
    );

    // 8. Create WiFi STA interface.
    let (controller, interfaces) =
        esp_radio::wifi::new(radio_init, peripherals.WIFI, Default::default()).unwrap();
    let sta_interface = interfaces.sta;

    // 9. Configure embassy-net with DHCP.
    let seed = {
        let rng = esp_hal::rng::Rng::new();
        (rng.random() as u64) | ((rng.random() as u64) << 32)
    };

    static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        sta_interface,
        embassy_net::Config::dhcpv4(Default::default()),
        RESOURCES.init(StackResources::new()),
        seed,
    );

    // 10. Spawn tasks.
    spawner.spawn(wifi_task(controller)).unwrap();
    spawner.spawn(net_task(runner)).unwrap();
    spawner.spawn(downlink_task(uart_rx, stack)).unwrap();
    spawner.spawn(uplink_task(uart_tx, stack)).unwrap();

    info!("cybesp-bridge: all tasks spawned");

    // Main task idles — all work happens in spawned tasks.
    loop {
        Timer::after_secs(60).await;
    }
}
