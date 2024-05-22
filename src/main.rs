#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;

use alloc::string::{String, ToString};
use alloc::vec;
use alloc::vec::Vec;
use core::alloc::{GlobalAlloc, Layout};
use core::cell::RefCell;
use core::fmt::Write;
use core::ptr::addr_of_mut;
use core::str::from_utf8_unchecked;

use display_interface_spi::SPIInterface;
use embassy_embedded_hal::SetConfig;
use embassy_embedded_hal::shared_bus::{asynch, blocking};
use embassy_executor::Spawner;
use embassy_net::{Config, Stack, StackResources};
use embassy_net::dns::DnsSocket;
use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_sync::{blocking_mutex, mutex};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::Drawable;
use embedded_graphics::geometry::{OriginDimensions, Point, Size};
use embedded_graphics::iterator::PixelIteratorExt;
use embedded_graphics::mono_font::{MonoFont, MonoTextStyle};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::{Alignment, Baseline, TextStyle, TextStyleBuilder};
use embedded_hal::spi::SpiDevice;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;
use embedded_iconoir::icons;
use embedded_iconoir::prelude::IconoirNewIcon;
use embedded_sdmmc::{DirEntry, SdCard};
use embedded_svc::io::asynch::BufRead;
use esp32_utils_crate::dummy_pin::DummyPin;
use esp32_utils_crate::fonts::CharacterStyles;
use esp32_utils_crate::ft6236_asynch::{FT6236, FT6236_DEFAULT_ADDR};
use esp32_utils_crate::ft6236_asynch::EventType::PressDown;
use esp32_utils_crate::graphics;
use esp32_utils_crate::graphics::{Button, GraphicUtils, Label, ListItem, Progress, Theme};
use esp32_utils_crate::sdcard::SdcardManager;
use esp32_utils_crate::tsc2007::TSC2007_ADDR;
use esp_backtrace;
use esp_hal::{Async, clock::ClockControl, embassy, get_core, peripherals::Peripherals, prelude::*, psram};
use esp_hal::clock::Clocks;
use esp_hal::cpu_control::CpuControl;
use esp_hal::dma::{DmaDescriptor, DmaPriority};
use esp_hal::dma::{Dma, I2s0DmaChannel};
use esp_hal::embassy::executor::Executor;
use esp_hal::gpio::{GpioPin, Input, Io, NO_PIN, Output, Pull};
use esp_hal::i2c::I2C;
use esp_hal::i2s::{DataFormat, I2s, Standard};
use esp_hal::i2s::asynch::I2sWriteDmaAsync;
use esp_hal::ledc::{channel, HighSpeed, Ledc, timer};
use esp_hal::peripherals::{I2C0, I2S0, SPI2};
use esp_hal::rng::Rng;
use esp_hal::spi::{FullDuplexMode, SpiMode};
use esp_hal::spi::master::{Instance, Spi};
use esp_hal::system::SystemControl;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use esp_wifi::{EspWifiInitFor, initialize};
use esp_wifi::wifi::{ClientConfiguration, Configuration, WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState};
use mipidsi::Builder;
use mipidsi::models::ILI9341Rgb565;
use mipidsi::options::{ColorOrder, Orientation, Rotation};
use profont::PROFONT_24_POINT;
use reqwless::client::HttpClient;
use reqwless::headers::ContentType::AudioMpeg;
use reqwless::request::{RequestBody, RequestBuilder};
use reqwless::request::Method::GET;
use reqwless::response::Status;
use rmp3::{MAX_SAMPLES_PER_FRAME, RawDecoder};
use rmp3::Frame;
use serde::{Deserialize, Serialize};
use static_cell::make_static;
use static_cell::StaticCell;

use crate::Error::NoError;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_psram_heap() {
    unsafe {
        ALLOCATOR.init(psram::PSRAM_VADDR_START as *mut u8, psram::PSRAM_BYTES);
    }
}

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

// in psram
const STREAM_BUFFER_SIZE: usize = 1024 * 32;
const STREAM_BUFFER_SIZE_MIN: usize = 1024 * 16;
const STREAM_BUFFER_REFILL_TIMEOUT: u64 = 5000;
// in micros
const RX_BUFFER_SIZE: usize = 1024 * 4;

const MAX_SAMPLES_PER_FRAME_BYTE: usize = MAX_SAMPLES_PER_FRAME * 2;

const I2S_BUFFER_SIZE: usize = 1024 * 16;

const SILENCE_DELAY_FRAMES: u64 = 128;

const SILENCE_SHORT_DELAY_FRAMES: u64 = 32;

const SCREEN_TIMEOUT_SECS: u64 = 30;
const SCREEN_BRIGHTNESS_PERCENT: u8 = 30;

static mut APP_CORE_STACK: esp_hal::cpu_control::Stack<8192> = esp_hal::cpu_control::Stack::new();

#[derive(Debug, Clone)]
struct MetaData {
    title: alloc::string::String,
    channels: u16,
    sample_rate: u32,
    bitrate: u32,
}

impl MetaData {
    fn is_incomplete(&self) -> bool {
        self.channels == 0 || self.sample_rate == 0 || self.bitrate == 0
    }
}

static META_DATA_SIGNAL: Signal<CriticalSectionRawMutex, MetaData> = Signal::new();

#[derive(Debug, Clone)]
struct StatsData {
    stream_bytes_per_sec: u64,
    send_bytes_per_sec: u64,
    receive_bytes_per_sec: u64,
}

static SEND_STATS_DATA_SIGNAL: Signal<CriticalSectionRawMutex, StatsData> = Signal::new();
static RECEIVE_STATS_DATA_SIGNAL: Signal<CriticalSectionRawMutex, StatsData> = Signal::new();


#[derive(Clone, Debug)]
struct ControlData {
    url: alloc::string::String,
    play: bool,
    pause: bool,
    change_url: bool,
}

impl ControlData {
    fn new_change_url(url: &str) -> Self {
        ControlData {
            url: String::from(url),
            play: true,
            pause: false,
            change_url: true,
        }
    }
    fn new_pause() -> Self {
        ControlData {
            url: String::from(""),
            play: false,
            pause: true,
            change_url: false,
        }
    }

    fn new_play(url: &str) -> Self {
        ControlData {
            url: String::from(url),
            play: true,
            pause: false,
            change_url: true,
        }
    }
}

static CONTROL_DATA_SIGNAL: Signal<CriticalSectionRawMutex, ControlData> = Signal::new();

#[derive(Clone, Copy)]
struct TouchData {
    x: u16,
    y: u16,
    z: u16,
}

impl TouchData {
    fn new(x: u16, y: u16, z: u16) -> Self {
        TouchData {
            x,
            y,
            z,
        }
    }
}

static TOUCH_DATA_SIGNAL: Signal<CriticalSectionRawMutex, TouchData> = Signal::new();

#[derive(Clone, Copy)]
struct FrameData {
    id: u64,
    data: [i16; MAX_SAMPLES_PER_FRAME],
}


impl FrameData {
    fn new() -> Self {
        FrameData {
            id: 0,
            data: [0i16; MAX_SAMPLES_PER_FRAME],
        }
    }
}

static FRAME_CHANNEL: Channel<CriticalSectionRawMutex, FrameData, 4> = Channel::new();

#[derive(Clone, Debug)]
enum Error {
    NoError,
    ConnectionError,
    ConnectionLost,
    DecoderError,
    FormatError,
}

fn get_error_description<'a>(error_type: Error) -> &'a str {
    match error_type {
        NoError => { "" }
        Error::ConnectionError => { "Connection error" }
        Error::ConnectionLost => { "Connection lost" }
        Error::DecoderError => { "Decoder error" }
        Error::FormatError => { "Stream format error" }
    }
}

#[derive(Clone, Debug)]
struct StatusData {
    paused: bool,
    url_index: usize,
    error: bool,
    error_type: Error,
    finished: bool,
}

impl StatusData {
    fn new_default() -> Self {
        StatusData {
            paused: true,
            url_index: 0,
            error: false,
            error_type: NoError,
            finished: false,
        }
    }

    fn new_error(error_type: Error) -> Self {
        StatusData {
            paused: false,
            url_index: 0,
            error: true,
            error_type,
            finished: false,
        }
    }

    fn new_finished() -> Self {
        StatusData {
            paused: false,
            url_index: 0,
            error: false,
            error_type: NoError,
            finished: true,
        }
    }
    fn set_paused(&mut self) {
        self.paused = true;
    }

    fn set_playing(&mut self, url_index: usize) {
        self.paused = false;
        self.url_index = url_index;
    }

    fn toggle_playing(&mut self) {
        self.paused = !self.paused;
    }
}

static STATUS_DATA_SIGNAL: Signal<CriticalSectionRawMutex, StatusData> = Signal::new();

#[derive(Clone, Debug, Serialize, Deserialize)]
struct RadioStation {
    url: alloc::string::String,
    title: alloc::string::String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct RadioStationList {
    list: Vec<RadioStation>,
}

impl ListItem for RadioStation {
    fn get_text(&self) -> String {
        self.title.clone()
    }

    fn get_height(&self) -> u16 {
        self.get_font().character_size.height as u16
    }
    fn get_width(&self, display_with: u32) -> u32 { display_with }

    fn get_font(&self) -> &MonoFont<'_> {
        &PROFONT_24_POINT
    }

    fn get_text_style(&self) -> TextStyle {
        TextStyleBuilder::new()
            .alignment(Alignment::Left)
            .baseline(Baseline::Top)
            .build()
    }
}

impl RadioStation {
    fn new(title: &str, url: &str) -> Self {
        RadioStation {
            url: String::from(url),
            title: String::from(title),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct MP3File {
    file_name: alloc::string::String,
}

impl ListItem for MP3File {
    fn get_text(&self) -> String {
        self.file_name.clone()
    }

    fn get_height(&self) -> u16 {
        self.get_font().character_size.height as u16
    }

    fn get_width(&self, display_with: u32) -> u32 { display_with }

    fn get_font(&self) -> &MonoFont<'_> {
        &PROFONT_24_POINT
    }

    fn get_text_style(&self) -> TextStyle {
        TextStyleBuilder::new()
            .alignment(Alignment::Left)
            .baseline(Baseline::Top)
            .build()
    }
}

impl MP3File {
    fn new(name: &str) -> Self {
        MP3File {
            file_name: String::from(name),
        }
    }
}

macro_rules! singleton {
    ($val:expr, $typ:ty) => {{
        static STATIC_CELL: StaticCell<$typ> = StaticCell::new();
        STATIC_CELL.init($val)
    }};
}

#[embassy_executor::task]
async fn net_task(stack: &'static embassy_net::Stack<WifiDevice<'static, WifiStaDevice>>) {
    stack.run().await
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    loop {
        match esp_wifi::wifi::get_wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: SSID.try_into().unwrap(),
                password: PASSWORD.try_into().unwrap(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            controller.start().await.unwrap();
            println!("Wifi started!");
        }

        match controller.connect().await {
            Ok(_) => println!("Wifi connected!"),
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

async fn send_custom_silence(short: bool) {
    let empty_frame_data = FrameData::new();
    let duration = if short { SILENCE_SHORT_DELAY_FRAMES } else { SILENCE_DELAY_FRAMES };
    for x in 0..duration {
        FRAME_CHANNEL.send(empty_frame_data).await;
    }
}

async fn send_silence() {
    send_custom_silence(false).await;
}

async fn send_short_silence() {
    send_custom_silence(true).await;
}

#[embassy_executor::task]
pub async fn handle_radio_stream(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    let mut current_url = alloc::string::String::new();
    let mut playback_paused = true;

    // let mut tls_read_buffer = [0; 4096];
    // let mut tls_write_buffer = [0; 4096];
    // let mut rx_buffer = [0; 4096];

    let data_buffer_layout = Layout::array::<u8>(STREAM_BUFFER_SIZE).unwrap();
    let data_buffer_ptr = unsafe { ALLOCATOR.alloc(data_buffer_layout) };
    let data_buffer = unsafe { core::slice::from_raw_parts_mut(data_buffer_ptr, data_buffer_layout.size()) };

    let rx_buffer_layout = Layout::array::<u8>(RX_BUFFER_SIZE).unwrap();
    let rx_buffer_ptr = unsafe { ALLOCATOR.alloc(rx_buffer_layout) };
    let rx_buffer = unsafe { core::slice::from_raw_parts_mut(rx_buffer_ptr, rx_buffer_layout.size()) };

    let mut frame_data = FrameData::new();

    let client_state = TcpClientState::<1, 4096, 4096>::new();
    let tcp_client = TcpClient::new(&stack, &client_state);
    let dns = DnsSocket::new(&stack);

    // let tls_config = TlsConfig::new(123456789u64, &mut tls_read_buffer, &mut tls_write_buffer, TlsVerify::None);
    let mut http_client = HttpClient::new(&tcp_client, &dns);
    loop {
        if playback_paused {
            println!("playback paused");
            let control_data = CONTROL_DATA_SIGNAL.wait().await;
            if control_data.change_url {
                current_url.clear();
                current_url.push_str(&control_data.url);
            }
            if control_data.play {
                playback_paused = false;
            }
            continue;
        }
        println!("connecting to {}", current_url);

        let url = alloc::string::String::from(current_url.as_str());
        let mut request = http_client.request(GET, url.as_str()).await.unwrap();
        let headers = [("Icy-MetaData", "1")];
        request = request.headers(&headers);

        let mut icy_metaint = 0;
        let response_result = request.send(rx_buffer).await;
        if response_result.is_err() {
            println!("error connecting");
            playback_paused = true;

            STATUS_DATA_SIGNAL.signal(StatusData::new_error(Error::ConnectionError));
            continue;
        }
        let response = response_result.unwrap();
        if response.status == Status::Ok {
            println!("connected to {}", current_url);

            if response.content_type != Some(AudioMpeg) {
                playback_paused = true;

                STATUS_DATA_SIGNAL.signal(StatusData::new_error(Error::FormatError));
                continue;
            }
            for i in response.headers() {
                let key = i.0;
                if key.starts_with("icy") {
                    println!("{}:{:?}", i.0, unsafe { from_utf8_unchecked(i.1) });
                    if key.eq_ignore_ascii_case("icy-metaint") {
                        let icy_metaint_str = unsafe { from_utf8_unchecked(i.1) };
                        icy_metaint = icy_metaint_str.parse().unwrap();
                    }
                }
            }

            if icy_metaint == 0 {
                println!("no icy stream");
                playback_paused = true;

                STATUS_DATA_SIGNAL.signal(StatusData::new_error(Error::FormatError));
                continue;
            } else {
                let mut decoder = RawDecoder::new();
                let mut body_reader = response.body().reader();
                let mut data_buffer_index = 0;
                let mut global_index = 0;
                let mut in_meta = false;
                let mut in_meta_data_index = 0;
                let mut meta_data_len = 0;
                let mut current_meta: Vec<u8> = vec![];
                let mut frame_id = 0u64;

                let mut meta_data = MetaData {
                    title: String::from(""),
                    bitrate: 0,
                    channels: 0,
                    sample_rate: 0,
                };
                let mut stats_data = StatsData {
                    stream_bytes_per_sec: 0,
                    send_bytes_per_sec: 0,
                    receive_bytes_per_sec: 0,
                };
                let mut stats_data_start = Instant::now();
                // let mut frame_channel_wait = false;
                let mut buffer_refill_start = Instant::now();
                let mut total_bytes_stream: u64 = 0;
                let mut total_bytes_send: u64 = 0;
                let stream_started = Instant::now();
                let mut send_started = Instant::now();

                'outer: loop {
                    if CONTROL_DATA_SIGNAL.signaled() {
                        let control_data = CONTROL_DATA_SIGNAL.wait().await;
                        println!("outer loop {:?}", control_data);
                        if control_data.change_url {
                            if current_url != control_data.url {
                                current_url.clear();
                                current_url.push_str(&control_data.url);

                                // send silence before change
                                send_silence().await;
                                break;
                            }
                        } else if control_data.pause {
                            playback_paused = true;

                            // send silence before pause
                            send_silence().await;
                            break;
                        }
                    }

                    match (body_reader.fill_buf().await) {
                        Ok(mut buf) => {
                            if buf.is_empty() {
                                println!("lost connection");
                                playback_paused = true;

                                STATUS_DATA_SIGNAL.signal(StatusData::new_error(Error::ConnectionLost));

                                // send silence before pause
                                send_silence().await;
                                break;
                            }
                            let buf_len = buf.len();
                            total_bytes_stream += buf_len as u64;
                            let mut buf_iter = buf.iter();
                            while let Some(b) = buf_iter.next() {
                                if !in_meta && global_index == icy_metaint {
                                    meta_data_len = (*b as u16 * 16) as usize;
                                    // println!("meta_data_len should be {}", meta_data_len);
                                    if meta_data_len != 0 {
                                        in_meta = true;
                                        current_meta.clear();
                                        in_meta_data_index = 0;
                                    } else {
                                        global_index = 0;
                                    }
                                    continue;
                                }
                                if in_meta {
                                    if *b != b'\0' {
                                        current_meta.push(*b);
                                    }
                                    in_meta_data_index = in_meta_data_index + 1;
                                    if in_meta_data_index == meta_data_len {
                                        let meta_data_str = unsafe { from_utf8_unchecked(&current_meta.as_slice()) };

                                        meta_data.title = String::from(meta_data_str);
                                        META_DATA_SIGNAL.signal(meta_data.clone());
                                        // println!("{}", meta_data_str);
                                        in_meta = false;
                                        global_index = 0;
                                    }
                                    continue;
                                } else {
                                    data_buffer[data_buffer_index] = *b;
                                    data_buffer_index += 1;

                                    // use max delay and not only full buffer
                                    if data_buffer_index == STREAM_BUFFER_SIZE ||
                                        (data_buffer_index > STREAM_BUFFER_SIZE_MIN && buffer_refill_start.elapsed().as_micros() > STREAM_BUFFER_REFILL_TIMEOUT) {
                                        // cant handle  so just shut up
                                        if meta_data.bitrate != 0 && meta_data.bitrate > 192 {
                                            println!("mp3 decoder error");
                                            playback_paused = true;

                                            STATUS_DATA_SIGNAL.signal(StatusData::new_error(Error::DecoderError));

                                            // send silence before pause
                                            send_silence().await;
                                            break 'outer;
                                        } else {
                                            let mut consumed = 0;
                                            let mut decoded_frames = 0;
                                            while let Some((frame, bytes_consumed)) = decoder.next(&data_buffer[consumed..data_buffer_index], &mut frame_data.data) {
                                                consumed += bytes_consumed;
                                                if let Frame::Audio(audio) = frame {
                                                    if meta_data.is_incomplete() {
                                                        meta_data.channels = audio.channels();
                                                        meta_data.sample_rate = audio.sample_rate();
                                                        meta_data.bitrate = audio.bitrate();
                                                        META_DATA_SIGNAL.signal(meta_data.clone());
                                                    }
                                                    if frame_id > SILENCE_DELAY_FRAMES {
                                                        frame_data.id = frame_id;
                                                        FRAME_CHANNEL.send(frame_data).await;
                                                        send_started = Instant::now();
                                                        total_bytes_send += (frame_data.data.len() * 2) as u64;
                                                    }
                                                    frame_id += 1;
                                                    decoded_frames += 1;
                                                    if data_buffer_index - consumed < STREAM_BUFFER_SIZE_MIN {
                                                        break;
                                                    }
                                                }
                                            }
                                            // move forward
                                            data_buffer_index -= consumed;
                                            for i in 0..data_buffer_index {
                                                data_buffer[i] = data_buffer[i + consumed];
                                            }
                                            buffer_refill_start = Instant::now();
                                        }
                                    }
                                    global_index = global_index + 1;
                                }
                            }
                            body_reader.consume(buf_len);
                        }
                        Err(e) => {
                            println!("{:?}", e);
                            break;
                        }
                    }
                    if stats_data_start.elapsed().as_secs() > 3 {
                        stats_data.stream_bytes_per_sec = total_bytes_stream / stream_started.elapsed().as_secs();
                        stats_data.send_bytes_per_sec = total_bytes_send / stream_started.elapsed().as_secs();
                        SEND_STATS_DATA_SIGNAL.signal(stats_data.clone());
                        stats_data_start = Instant::now();
                    }
                }
            }
        } else {
            println!("connect response.status {:?}", response.status);

            playback_paused = true;

            STATUS_DATA_SIGNAL.signal(StatusData::new_error(Error::ConnectionError));
        }
    }
}

#[embassy_executor::task]
pub async fn handle_frame_stream(i2s: I2s<'static, I2S0, I2s0DmaChannel, Async, >, bclk_pin: GpioPin<26>, ws_pin: GpioPin<25>,
                                 dout_pin: GpioPin<27>) {
    let i2s_tx = i2s
        .i2s_tx
        .with_bclk(bclk_pin)
        .with_ws(ws_pin)
        .with_dout(dout_pin)
        .build();

    let tx_buffer = make_static!([0u8; I2S_BUFFER_SIZE]);

    let mut transaction = i2s_tx.write_dma_circular_async(tx_buffer).unwrap();
    let mut frame_id: u64 = 0;
    let mut stats_data = StatsData {
        stream_bytes_per_sec: 0,
        send_bytes_per_sec: 0,
        receive_bytes_per_sec: 0,
    };
    let mut stats_start_init = false;
    let mut stats_data_start = Instant::now();
    let mut total_bytes: u64 = 0;
    let mut receive_data_start = Instant::now();

    loop {
        let frame_data = FRAME_CHANNEL.receive().await;
        if !stats_start_init {
            stats_data.receive_bytes_per_sec = 0;
            stats_data_start = Instant::now();
            receive_data_start = Instant::now();
            stats_start_init = true;
        }
        frame_id = frame_data.id;
        // println!("frame received frame_id = {}", frame_id);

        let frame_data_part = frame_data.data;
        let frame_data_part_u8 =
            unsafe { core::slice::from_raw_parts(&frame_data_part as *const _ as *const u8, frame_data_part.len() * 2) };
        let mut written_bytes = 0;
        while written_bytes != frame_data_part_u8.len() {
            match transaction.push(&frame_data_part_u8[written_bytes..]).await {
                Ok(written) => {
                    written_bytes += written;
                }
                Err(e) => { println!("transaction.push error = {:?}", e) }
            }
        }
        total_bytes += frame_data_part_u8.len() as u64;
        if stats_data_start.elapsed().as_secs() > 3 {
            stats_data.receive_bytes_per_sec = total_bytes / receive_data_start.elapsed().as_secs();
            RECEIVE_STATS_DATA_SIGNAL.signal(stats_data.clone());
            stats_data_start = Instant::now();
        }
    }
}

#[embassy_executor::task]
async fn handle_tp_touch_ft6206(i2c: asynch::i2c::I2cDevice<'static, CriticalSectionRawMutex, I2C<'static, I2C0, Async>>, tp_irq_pin: GpioPin<32>,
                                orientation: Rotation, width: u16, height: u16) {
    let mut ft6206 = FT6236::new(i2c);
    let mut tp_irq_input = Input::new(tp_irq_pin, Pull::Down);

    loop {
        tp_irq_input.wait_for_low().await;
        if let Ok(point) = ft6206.get_point0().await {
            if point.is_some() {
                let point_event = point.unwrap();
                let x = point_event.x;
                let y = point_event.y;
                let z = point_event.weight;
                if point_event.event == PressDown {
                    let (x, y) = match orientation {
                        Rotation::Deg0 => (width - x, height - y),
                        Rotation::Deg90 => (width - y, x),
                        Rotation::Deg180 => (x, y),
                        Rotation::Deg270 => (y, height - x)
                    };
                    println!("{}x{}", x, y);

                    TOUCH_DATA_SIGNAL.signal(TouchData::new(x, y, z.into()));
                    // debounce
                    Timer::after(Duration::from_millis(100)).await
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn handle_play_mp3_from_sd(sdcard_manager: &'static mut SdcardManager<SdCard<blocking::spi::SpiDevice<'static, CriticalSectionRawMutex, Spi<'static, SPI2, FullDuplexMode>, DummyPin>, Output<'static, GpioPin<14>>, Delay>>) {
    let data_buffer_layout = Layout::array::<u8>(STREAM_BUFFER_SIZE).unwrap();
    let data_buffer_ptr = unsafe { ALLOCATOR.alloc(data_buffer_layout) };
    let data_buffer = unsafe { core::slice::from_raw_parts_mut(data_buffer_ptr, data_buffer_layout.size()) };

    let data_chunk_layout = Layout::array::<u8>(1024 * 4).unwrap();
    let data_chunk_ptr = unsafe { ALLOCATOR.alloc(data_chunk_layout) };
    let data_chunk = unsafe { core::slice::from_raw_parts_mut(data_chunk_ptr, data_chunk_layout.size()) };

    let mut frame_data = FrameData::new();

    let mut playback_paused = true;
    let mut current_file = alloc::string::String::new();

    'outer: loop {
        if playback_paused {
            println!("playback paused");
            let control_data = CONTROL_DATA_SIGNAL.wait().await;
            if control_data.change_url {
                current_file.clear();
                current_file.push_str(&control_data.url);
            }
            if control_data.play {
                playback_paused = false;
            }
            continue;
        }

        sdcard_manager.open_root_dir().unwrap();
        if let Ok(mp3_file) = sdcard_manager.open_file_in_root_dir_for_reading(current_file.as_str()) {
            println!("Start playing mp3 {}", current_file);
            let mut decoder = RawDecoder::new();
            let mut meta_data = MetaData {
                title: String::from(""),
                bitrate: 0,
                channels: 0,
                sample_rate: 0,
            };

            let mut frame_id = 0;
            let mut data_buffer_index = 0;
            let mut eof = false;
            let mut chunk_remaining = 0;
            let mut bytes_read = 0;
            let mut chunk_used = 0;
            let mut finished = false;

            loop {
                if CONTROL_DATA_SIGNAL.signaled() {
                    let control_data = CONTROL_DATA_SIGNAL.wait().await;
                    println!("outer loop {:?}", control_data);
                    if control_data.change_url {
                        if current_file != control_data.url {
                            current_file.clear();
                            current_file.push_str(&control_data.url);

                            // send silence before change
                            send_short_silence().await;
                            break;
                        }
                    } else if control_data.pause {
                        playback_paused = true;

                        // send silence before pause
                        send_short_silence().await;
                        break;
                    }
                }
                while data_buffer_index < STREAM_BUFFER_SIZE {
                    if chunk_remaining == 0 && !eof {
                        // println!("load new chunk data_buffer_index = {}", data_buffer_index);
                        if let Ok(num_read) = sdcard_manager.load_open_file_into_buffer(mp3_file, data_chunk) {
                            if num_read == 0 {
                                eof = true;
                                break;
                            }
                            bytes_read += num_read;
                            chunk_remaining = num_read;
                            chunk_used = 0;
                        }
                    }

                    // println!("refill from chunk data_buffer_index = {} size {}", data_buffer_index, chunk_remaining.min(data_buffer.len() - data_buffer_index));

                    let mut chunk_read = 0;
                    for i in 0..chunk_remaining.min(data_buffer.len() - data_buffer_index) {
                        data_buffer[data_buffer_index + i] = data_chunk[chunk_used + i];
                        chunk_read += 1
                    }
                    chunk_used += chunk_read;
                    chunk_remaining -= chunk_read;
                    data_buffer_index += chunk_read;
                    // println!("{} {} {} {}", chunk_read, chunk_used, chunk_remaining, data_buffer_index);
                }

                let mut consumed = 0;
                while let Some((frame, bytes_consumed)) = decoder.next(&data_buffer[consumed..data_buffer_index], &mut frame_data.data) {
                    // println!("{}", data_buffer_index- consumed);
                    consumed += bytes_consumed;
                    if let Frame::Audio(audio) = frame {
                        if meta_data.is_incomplete() {
                            meta_data.channels = audio.channels();
                            meta_data.sample_rate = audio.sample_rate();
                            meta_data.bitrate = audio.bitrate();
                            META_DATA_SIGNAL.signal(meta_data.clone());
                        }
                        frame_data.id = frame_id;
                        // println!("samples = {}", audio.sample_count() * audio.channels());
                        FRAME_CHANNEL.send(frame_data).await;
                        frame_id += 1;
                    } else if let Frame::Other(data) = frame {
                        // TODO i3 tag parser?
                        // println!("id3 tags {}", unsafe { from_utf8_unchecked(data) });
                        // meta_data.title = unsafe { String::from(from_utf8_unchecked(&data[..256])) };
                        // META_DATA_SIGNAL.signal(meta_data.clone());
                    }
                    if data_buffer_index - consumed < STREAM_BUFFER_SIZE_MIN && !eof {
                        // println!("stop decode at {}", data_buffer_index - consumed);
                        break;
                    }
                }
                // println!("consumed = {}", consumed);
                // move forward
                data_buffer_index -= consumed;
                for i in 0..data_buffer_index {
                    data_buffer[i] = data_buffer[i + consumed];
                }

                if data_buffer_index == 0 && eof {
                    println!("finished bytes_read = {}", bytes_read);
                    finished = true;
                    playback_paused = true;

                    send_short_silence().await;
                    break;
                }
            }

            let res = sdcard_manager.close_open_file(mp3_file);
            if res.is_err() {
                println!("close file error {:?}", res.err().unwrap());
            }
            sdcard_manager.close_root_dir().unwrap();

            if finished {
                STATUS_DATA_SIGNAL.signal(StatusData::new_finished());
            }
        } else {
            println!("Failed to play mp3 file {}", current_file.as_str());
            sdcard_manager.close_root_dir().unwrap();
            playback_paused = true;
        }
    }
}

fn get_left_button_pos(width: u32, height: u32) -> Point {
    Point::new(0, (height - GraphicUtils::get_button_size().height) as i32)
}

fn get_right_button_pos(width: u32, height: u32) -> Point {
    Point::new((width - GraphicUtils::get_button_size().width) as i32, (height - GraphicUtils::get_button_size().height) as i32)
}

fn get_middle_button_pos(width: u32, height: u32) -> Point {
    Point::new((width / 2 - GraphicUtils::get_button_size().width / 2) as i32, (height - GraphicUtils::get_button_size().height) as i32)
}

fn display_play_navigation<D>(display: &mut D, width: u32, height: u32, status: &StatusData, theme: &Theme) -> Result<(), D::Error>
    where D: DrawTarget<Color=Rgb565> {
    let color = theme.button_foreground_color;
    let icon_next = icons::size24px::navigation::ArrowRight::new(color);
    let icon_play = icons::size24px::music::Play::new(color);
    let icon_pause = icons::size24px::music::Pause::new(color);
    let icon_list = icons::size24px::layout::TableRows::new(color);

    let image_next = Button::new(&icon_next, get_left_button_pos(width, height));
    let image_play = Button::new(&icon_play, get_right_button_pos(width, height));
    let image_pause = Button::new(&icon_pause, get_right_button_pos(width, height));
    let image_list = Button::new(&icon_list, get_middle_button_pos(width, height));

    let background_style = PrimitiveStyleBuilder::new()
        .fill_color(theme.button_background_color)
        .build();

    if status.paused {
        image_play.draw(display, background_style)?;
    } else {
        image_pause.draw(display, background_style)?;
    }
    image_next.draw(display, background_style)?;
    image_list.draw(display, background_style)
}

fn display_list_navigation<D>(display: &mut D, width: u32, height: u32, theme: &Theme) -> Result<(), D::Error>
    where D: DrawTarget<Color=Rgb565> {
    let color = theme.button_foreground_color;
    let icon_up = icons::size24px::navigation::ArrowUp::new(color);
    let icon_down = icons::size24px::navigation::ArrowDown::new(color);
    let icon_select = icons::size24px::music::Play::new(color);

    let image_down = Button::new(&icon_up, get_left_button_pos(width, height));
    let image_up = Button::new(&icon_down, get_right_button_pos(width, height));
    let image_select = Button::new(&icon_select, get_middle_button_pos(width, height));

    let background_style = PrimitiveStyleBuilder::new()
        .fill_color(theme.button_background_color)
        .build();
    image_down.draw(display, background_style)?;
    image_up.draw(display, background_style)?;
    image_select.draw(display, background_style)
}

fn display_mode_navigation<D>(display: &mut D, width: u32, height: u32, theme: &Theme) -> Result<(), D::Error>
    where D: DrawTarget<Color=Rgb565> {
    let color = theme.button_foreground_color;
    let icon_radio = icons::size24px::connectivity::Wifi::new(color);
    let icon_sd = icons::size24px::devices::HardDrive::new(color);

    let image_radio = Button::new(&icon_radio, get_left_button_pos(width, height));
    let image_sd = Button::new(&icon_sd, get_right_button_pos(width, height));

    let background_style = PrimitiveStyleBuilder::new()
        .fill_color(theme.button_background_color)
        .build();
    image_radio.draw(display, background_style)?;
    image_sd.draw(display, background_style)
}

pub fn from_ascii(bytes: &[u8]) -> Result<&str, &'static str> {
    if bytes.iter().all(|b| *b < 128) {
        Ok(unsafe { core::str::from_utf8_unchecked(bytes) })
    } else {
        Err("Not an ascii!")
    }
}

#[embassy_executor::task]
async fn foo() {
    loop {
        println!("Running foo() on core {}", get_core() as usize);
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();

    psram::init_psram(peripherals.PSRAM);
    init_psram_heap();

    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = singleton!(
        ClockControl::max(system.clock_control).freeze(),
        Clocks
    );

    let timer_group0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0);

    let timer0 = TimerGroup::new(peripherals.TIMG1, &clocks, None).timer0;
    let mut rng = Rng::new(peripherals.RNG);
    let _init = initialize(
        EspWifiInitFor::Wifi,
        timer0,
        rng,
        peripherals.RADIO_CLK,
        &clocks,
    ).unwrap();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let bl_pin = io.pins.gpio4;
    let i2c_power_pin = io.pins.gpio2;
    let i2c_sda_pin = io.pins.gpio22;
    let i2c_scl_pin = io.pins.gpio20;
    let tp_irq_pin = io.pins.gpio32;
    let spi_miso_pin = io.pins.gpio21;
    let spi_mosi_pin = io.pins.gpio19;
    let spi_scl_pin = io.pins.gpio5;
    let spi_tft_dc_pin = io.pins.gpio33;
    let spi_tft_cs_pin = io.pins.gpio15;
    let spi_sd_cs_pin = io.pins.gpio14;

    // backlight
    let mut ledc = Ledc::new(
        peripherals.LEDC,
        &clocks);

    let mut hstimer0 = ledc.get_timer::<HighSpeed>(timer::Number::Timer0);
    hstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty5Bit,
            clock_source: timer::HSClockSource::APBClk,
            frequency: 24u32.kHz(),
        })
        .unwrap();

    let mut channel0 = ledc.get_channel(channel::Number::Channel0, bl_pin);
    channel0
        .configure(channel::config::Config {
            timer: &hstimer0,
            duty_pct: SCREEN_BRIGHTNESS_PERCENT,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    // enable i2c_power
    Output::new(i2c_power_pin, true);

    let i2c0 = I2C::new_async(
        peripherals.I2C0,
        i2c_sda_pin,
        i2c_scl_pin,
        40u32.kHz(),
        clocks,
    );

    let i2c0_bus = mutex::Mutex::<blocking_mutex::raw::CriticalSectionRawMutex, _>::new(i2c0);
    let i2c0_bus_static = make_static!(i2c0_bus);

    let mut i2c0_dev0 = asynch::i2c::I2cDevice::new(i2c0_bus_static);
    let has_tsc2007 = i2c0_dev0.read(TSC2007_ADDR, &mut [0]).await.is_ok();
    println!("has_tsc2007 = {}", has_tsc2007);
    let has_ft6206 = i2c0_dev0.read(FT6236_DEFAULT_ADDR, &mut [0]).await.is_ok();
    println!("has_ft6206 = {}", has_ft6206);

    let sclk = spi_scl_pin;
    let miso = spi_miso_pin;
    let mosi = spi_mosi_pin;
    let dc = Output::new(spi_tft_dc_pin, false);

    let display_cs = Output::new(spi_tft_cs_pin, false);
    let sd_cs = Output::new(spi_sd_cs_pin, false);

    let spi2 = Spi::new(peripherals.SPI2, 20u32.MHz(), SpiMode::Mode0, clocks)
        .with_pins(Some(sclk), Some(mosi), Some(miso), NO_PIN);

    let spi2_bus = blocking_mutex::Mutex::<blocking_mutex::raw::CriticalSectionRawMutex, _>::new(RefCell::new(spi2));
    let spi2_bus_static = make_static!(spi2_bus);

    let display_spi = blocking::spi::SpiDevice::new(
        spi2_bus_static,
        display_cs);
    let sd_spi = blocking::spi::SpiDevice::new(
        spi2_bus_static,
        DummyPin);

    let spi_iface = SPIInterface::new(display_spi, dc);

    let mut delay = Delay;

    let mut display = Builder::new(ILI9341Rgb565, spi_iface)
        .orientation(Orientation::new().rotate(Rotation::Deg270).flip_horizontal())
        .color_order(ColorOrder::Bgr)
        .display_size(240, 320)
        .init(&mut delay).unwrap();

    let display_width = display.size().width;
    let display_height = display.size().height;
    let display_rotation = display.orientation().rotation;

    let theme = Theme::new_light_theme();
    let mut character_styles = CharacterStyles::new_with_color(theme.text_color_primary);
    character_styles.set_background_color(theme.screen_background_color);

    let icon_left_area = Rectangle::new(get_left_button_pos(display_width, display_height), GraphicUtils::get_button_size());
    let icon_right_area = Rectangle::new(get_right_button_pos(display_width, display_height), GraphicUtils::get_button_size());
    let icon_middle_area = Rectangle::new(get_middle_button_pos(display_width, display_height), GraphicUtils::get_button_size());

    let mut status = StatusData::new_default();

    let mut meta_data = MetaData {
        title: String::from(""),
        bitrate: 0,
        channels: 2,
        sample_rate: 0,
    };

    let wifi_icon = icons::size96px::connectivity::Wifi::new(theme.text_color_primary);
    let mut progress = Progress::new(&wifi_icon, "Connecting to Wifi...",
                                     Point::new(0, 0), Size::new(display_width, display_height),
                                     theme.screen_background_color, character_styles.default_character_style(),
                                     &theme);

    let sdcard = SdCard::new(sd_spi, sd_cs, Delay);
    let mut sdcard_manager = make_static!(SdcardManager::new(sdcard));

    let mut root_file_list: Vec<DirEntry> = Vec::new();
    if let Ok(()) = sdcard_manager.open_root_dir() {
        match sdcard_manager.get_root_dir_entries(&mut root_file_list) {
            Ok(()) => {}
            Err(e) => println!("{:?}", e)
        }
    }
    let mut station_list: Vec<RadioStation> = Vec::new();
    let mut mp3file_list: Vec<MP3File> = Vec::new();

    for file in &root_file_list {
        if file.name.to_string() == "RADIO.JSO" {
            let mut buffer_vec = vec![0; file.size as usize];
            let load_result = sdcard_manager.load_root_dir_file_into_buffer("RADIO.JSO", &mut buffer_vec);
            if load_result.is_ok() {
                let mut sd_station_list = serde_json::from_slice::<RadioStationList>(buffer_vec.as_slice()).unwrap().list;
                station_list.append(&mut sd_station_list);
            }
        } else if from_ascii(file.name.extension()).unwrap() == "MP3" {
            mp3file_list.push(MP3File::new(&file.name.to_string()));
        }
    }
    sdcard_manager.close_root_dir().unwrap();

    // let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    // let _guard = cpu_control
    //     .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, move || {
    //         static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    //         let executor = EXECUTOR.init(Executor::new());
    //         executor.run(|spawner| {
    //             spawner.spawn(foo()).ok();
    //         });
    //     })
    //     .unwrap();

    progress.update_text(&mut display, "Connecting to Wifi...").unwrap();

    let wifi = peripherals.WIFI;
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&_init, wifi, WifiStaDevice).unwrap();

    let config = Config::dhcpv4(Default::default());

    // Init network stack
    let net_stack = Stack::new(
        wifi_interface,
        config,
        make_static!(StackResources::<3>::new()),
        rng.random() as u64,
    );
    let stack = make_static!(net_stack);

    // TODO connection can throw errors - how to show in progress?
    spawner.must_spawn(connection(controller));
    spawner.must_spawn(net_task(stack));

    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    progress.update_text(&mut display, "Waiting for IP address...").unwrap();

    loop {
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    display.clear(theme.screen_background_color).unwrap();

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.i2s0channel;

    // from dma_circular_buffers macro
    let tx_descriptors = make_static!([DmaDescriptor::EMPTY; (I2S_BUFFER_SIZE + 4091) / 4092]);
    let rx_descriptors = make_static!([DmaDescriptor::EMPTY; 0]);

    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data16Channel16,
        44100u32.Hz(),
        dma_channel.configure_for_async(
            false,
            tx_descriptors,
            rx_descriptors,
            DmaPriority::Priority0,
        ),
        clocks,
    );

    if station_list.len() == 0 {
        // station_list.push(RadioStation::new("FM4", "http://orf-live.ors-shoutcast.at/fm4-q2a"));
        // station_list.push(RadioStation::new("MetalRock.FM", "http://cheetah.streemlion.com:2160/stream"));
        // station_list.push(RadioStation::new("Terry Callaghan's Classic Alternative Channel", "http://s16.myradiostream.com:7304"));
        // station_list.push(RadioStation::new("OE3", "http://orf-live.ors-shoutcast.at/oe3-q2a"));
        station_list.push(RadioStation::new("local", "http://192.168.1.70:8001/stream"));
    }

    let mut mp3file_mode = if mp3file_list.len() != 0 { true } else { false };
    let mut current_screen = -1;

    println!("{:?}", station_list);

    let mut radio_select_list = graphics::List::new(&station_list, Point::new(10, 10),
                                                    Size::new(display_width - 10, (display_height - GraphicUtils::get_button_size().height)),
                                                    &theme);
    if !mp3file_mode {
        current_screen = 0;
        spawner.must_spawn(handle_radio_stream(stack));
        radio_select_list.draw(&mut display).unwrap();
        display_list_navigation(&mut display, display_width, display_height, &theme).unwrap();
    }

    println!("{:?}", mp3file_list);

    let mut mp3file_select_list = graphics::List::new(&mp3file_list, Point::new(10, 10),
                                                      Size::new(display_width - 10, (display_height - GraphicUtils::get_button_size().height)),
                                                      &theme);
    if mp3file_mode {
        current_screen = -1;
        display_mode_navigation(&mut display, display_width, display_height, &theme).unwrap();
        // mp3file_select_list.draw(&mut display).unwrap();
    }

    spawner.must_spawn(handle_frame_stream(i2s, io.pins.gpio26, io.pins.gpio25, io.pins.gpio27));
    // if !mp3file_mode {
    //     spawner.must_spawn(handle_radio_stream(stack));
    // }
    // if mp3file_mode {
    //     spawner.must_spawn(handle_play_mp3_from_sd(sdcard_manager));
    // }

    if has_ft6206 {
        spawner.must_spawn(handle_tp_touch_ft6206(i2c0_dev0, tp_irq_pin, display_rotation, display_width as u16, display_height as u16));
    }

    // let mut file_written = 0;
    // let mut file_done = false;
    // let mut time = Instant::now();
    let mut current_sample_rate = 44100u32;

    let error_character_style = MonoTextStyle::new(
        character_styles.medium_character_style().font,
        theme.error_color);

    // let title_style = PrimitiveStyleBuilder::new()
    //     .fill_color(Rgb565::MAGENTA)
    //     .build();
    // let stats_style = PrimitiveStyleBuilder::new()
    //     .fill_color(Rgb565::GREEN)
    //     .build();
    // let artist_style = PrimitiveStyleBuilder::new()
    //     .fill_color(Rgb565::BLUE)
    //     .build();

    let title_line_pos = Point::new(10, 10);
    let meta_first_line_pos = Point::new(10, 40);
    let meta_second_line_pos = Point::new(10, 60);
    let format_line_pos = Point::new(10, 80);
    let error_line_pos = Point::new(10, 120);

    let mut title_label = Label::new(" ", title_line_pos, display_width, theme.screen_background_color, character_styles.large_character_style(), &theme);
    let mut meta_first_label = Label::new(" ", meta_first_line_pos, display_width, theme.screen_background_color, character_styles.medium_character_style(), &theme);
    let mut meta_second_label = Label::new(" ", meta_second_line_pos, display_width, theme.screen_background_color, character_styles.medium_character_style(), &theme);
    let mut format_label = Label::new(" ", format_line_pos, display_width, theme.screen_background_color, character_styles.medium_character_style(), &theme);
    let mut error_label = Label::new(" ", error_line_pos, display_width, theme.screen_background_color, error_character_style, &theme);

    if current_screen == -1 {
        let touch_data = TOUCH_DATA_SIGNAL.wait().await;
        let touch_point = Point::new(touch_data.x as i32, touch_data.y as i32);

        if icon_left_area.contains(touch_point) {
            mp3file_mode = false;
            current_screen = 0;
            spawner.must_spawn(handle_radio_stream(stack));
            radio_select_list.draw(&mut display).unwrap();
            display_list_navigation(&mut display, display_width, display_height, &theme).unwrap();
        } else if icon_right_area.contains(touch_point) {
            mp3file_mode = true;
            current_screen = 0;
            spawner.must_spawn(handle_play_mp3_from_sd(sdcard_manager));
            mp3file_select_list.draw(&mut display).unwrap();
            display_list_navigation(&mut display, display_width, display_height, &theme).unwrap();
        }
    }

    let mut display_off = false;
    let mut time = Instant::now();

    loop {
        if META_DATA_SIGNAL.signaled() {
            meta_data = META_DATA_SIGNAL.wait().await;

            if current_screen == 1 {
                display_play_navigation(&mut display, display_width, display_height, &status, &theme).unwrap();

                // clear error
                error_label.update_text(&mut display, " ").unwrap();

                if mp3file_mode {
                    title_label.update_text(&mut display, mp3file_list[status.url_index].file_name.as_str()).unwrap();
                } else {
                    title_label.update_text(&mut display, station_list[status.url_index].title.as_str()).unwrap();
                }

                let mut buf = alloc::string::String::new();

                let mut title = meta_data.title.strip_prefix("StreamTitle='").unwrap_or(meta_data.title.as_str());
                title = title.strip_suffix("';").unwrap_or(meta_data.title.as_str());

                let split_pos = title.find(" - ").unwrap_or(0);
                if split_pos != 0 {
                    let (artist, track) = title.split_at(split_pos);
                    meta_first_label.update_text(&mut display, artist).unwrap();
                    meta_second_label.update_text(&mut display, track.strip_prefix(" - ").unwrap_or(track)).unwrap();
                } else {
                    meta_first_label.update_text(&mut display, title).unwrap();
                    meta_second_label.update_text(&mut display, " ").unwrap();
                }
                if meta_data.sample_rate != 0 {
                    buf.clear();
                    write!(buf, "{} / {} / {}", meta_data.sample_rate, meta_data.channels, meta_data.bitrate).unwrap();
                    format_label.update_text(&mut display, buf.as_str()).unwrap();
                } else {
                    format_label.update_text(&mut display, " ").unwrap();
                }
            }
            if meta_data.sample_rate != 0 {
                if meta_data.sample_rate != current_sample_rate {
                    println!("change sample rate to {}", meta_data.sample_rate);
                    current_sample_rate = meta_data.sample_rate;
                    esp_hal::i2s::private::update_config(Standard::Philips,
                                                         DataFormat::Data16Channel16,
                                                         meta_data.sample_rate.Hz(),
                                                         clocks);
                }
            }
        }
        if SEND_STATS_DATA_SIGNAL.signaled() {
            let stats_data = SEND_STATS_DATA_SIGNAL.wait().await;
            println!("send: {:?}", stats_data);
        }
        if RECEIVE_STATS_DATA_SIGNAL.signaled() {
            let stats_data = RECEIVE_STATS_DATA_SIGNAL.wait().await;
            println!("receive: {:?}", stats_data);
        }

        if TOUCH_DATA_SIGNAL.signaled() {
            time = Instant::now();

            let touch_data = TOUCH_DATA_SIGNAL.wait().await;
            let touch_point = Point::new(touch_data.x as i32, touch_data.y as i32);

            if display_off {
                if let Ok(()) = channel0.set_duty(SCREEN_BRIGHTNESS_PERCENT) {
                    display_off = false;
                    continue;
                }
            }
            if icon_left_area.contains(touch_point) {
                if current_screen == 0 {
                    if mp3file_mode {
                        mp3file_select_list.scroll_up(&mut display).unwrap();
                    } else {
                        radio_select_list.scroll_up(&mut display).unwrap();
                    }
                } else if current_screen == 1 {
                    println!("control change_url");
                    let url_index_new = if mp3file_mode { (status.url_index + 1) % mp3file_list.len() } else {
                        (status.url_index + 1) % station_list.len()
                    };
                    status.set_playing(url_index_new);
                    if mp3file_mode {
                        mp3file_select_list.set_selected_index(url_index_new);
                        CONTROL_DATA_SIGNAL.signal(ControlData::new_change_url(&mp3file_list[url_index_new].file_name));
                    } else {
                        radio_select_list.set_selected_index(url_index_new);
                        CONTROL_DATA_SIGNAL.signal(ControlData::new_change_url(&station_list[url_index_new].url));
                    }
                }
            }
            if icon_right_area.contains(touch_point) {
                if current_screen == 0 {
                    if mp3file_mode {
                        mp3file_select_list.scroll_down(&mut display).unwrap();
                    } else {
                        radio_select_list.scroll_down(&mut display).unwrap();
                    }
                } else if current_screen == 1 {
                    println!("control play_pause");
                    status.toggle_playing();
                    if status.paused {
                        CONTROL_DATA_SIGNAL.signal(ControlData::new_pause());
                    } else {
                        if mp3file_mode {
                            CONTROL_DATA_SIGNAL.signal(ControlData::new_play(&mp3file_list[status.url_index].file_name));
                        } else {
                            CONTROL_DATA_SIGNAL.signal(ControlData::new_play(&station_list[status.url_index].url));
                        }
                    }
                    display_play_navigation(&mut display, display_width, display_height, &status, &theme).unwrap();
                }
            }
            if icon_middle_area.contains(touch_point) {
                if current_screen == 0 {
                    current_screen = 1;
                    display.clear(theme.screen_background_color).unwrap();
                    // show current meta data
                    META_DATA_SIGNAL.signal(meta_data.clone());

                    let url_index_new = if mp3file_mode { mp3file_select_list.get_selected_index() } else { radio_select_list.get_selected_index() };
                    status.set_playing(url_index_new);
                    if mp3file_mode {
                        CONTROL_DATA_SIGNAL.signal(ControlData::new_change_url(&mp3file_list[url_index_new].file_name));
                    } else {
                        CONTROL_DATA_SIGNAL.signal(ControlData::new_change_url(&station_list[url_index_new].url));
                    }
                    display_play_navigation(&mut display, display_width, display_height, &status, &theme).unwrap();
                } else if current_screen == 1 {
                    current_screen = 0;
                    display.clear(theme.screen_background_color).unwrap();

                    if mp3file_mode {
                        mp3file_select_list.draw(&mut display).unwrap();
                    } else {
                        radio_select_list.draw(&mut display).unwrap();
                    }
                    display_list_navigation(&mut display, display_width, display_height, &theme).unwrap();
                }
            }
            if current_screen == 0 {
                if mp3file_mode {
                    if mp3file_select_list.get_bounding_box().contains(touch_point) {
                        if let Ok(selected_index) = mp3file_select_list.select_at_pos(&mut display, touch_point) {
                            current_screen = 1;
                            display.clear(theme.screen_background_color).unwrap();
                            // show current meta data
                            META_DATA_SIGNAL.signal(meta_data.clone());

                            status.set_playing(selected_index);
                            CONTROL_DATA_SIGNAL.signal(ControlData::new_change_url(&mp3file_list[selected_index].file_name));

                            display_play_navigation(&mut display, display_width, display_height, &status, &theme).unwrap();
                        }
                    }
                } else {
                    if radio_select_list.get_bounding_box().contains(touch_point) {
                        if let Ok(selected_index) = radio_select_list.select_at_pos(&mut display, touch_point) {
                            current_screen = 1;
                            display.clear(theme.screen_background_color).unwrap();
                            // show current meta data
                            META_DATA_SIGNAL.signal(meta_data.clone());

                            status.set_playing(selected_index);
                            CONTROL_DATA_SIGNAL.signal(ControlData::new_change_url(&station_list[selected_index].url));

                            display_play_navigation(&mut display, display_width, display_height, &status, &theme).unwrap();
                        }
                    }
                }
            }
        }
        if STATUS_DATA_SIGNAL.signaled() {
            let status_in = STATUS_DATA_SIGNAL.wait().await;
            if status_in.error {
                status.set_paused();
                if current_screen == 1 {
                    error_label.update_text(&mut display, get_error_description(status_in.error_type)).unwrap();
                    display_play_navigation(&mut display, display_width, display_height, &status, &theme).unwrap();
                }
            }
            if status_in.finished {
                if mp3file_mode {
                    println!("next mp3 track");
                    let url_index_new = (status.url_index + 1) % mp3file_list.len();
                    status.set_playing(url_index_new);
                    mp3file_select_list.set_selected_index(url_index_new);
                    CONTROL_DATA_SIGNAL.signal(ControlData::new_change_url(&mp3file_list[url_index_new].file_name));
                }
            }
        }

        // let frame_data = FRAME_CHANNEL.receive().await;
        // let mut frame_index = 0;
        //
        // while frame_index < MAX_SAMPLE_PER_FRAME_DATA {
        //     let frame_data_part = frame_data.data[frame_index];
        //     let frame_data_part_u8 =
        //         unsafe { core::slice::from_raw_parts(&frame_data_part as *const _ as *const u8, frame_data_part.len() * 2) };
        //
        //     // if !file_done {
        //     //     if file_written < 4000 {
        //     //         sdcard_manager.write_file_in_root_dir_from_buffer(file, frame_data_part_u8).unwrap();
        //     //         file_written += 1;
        //     //     } else {
        //     //         sdcard_manager.close_file(file).unwrap();
        //     //         println!("close file");
        //     //         sdcard_manager.close_root_dir().unwrap();
        //     //         println!("close dir");
        //     //         file_done = true;
        //     //     }
        //     // }
        //     frame_index += 1;
        // }
        if !display_off {
            if time.elapsed().as_secs() >= SCREEN_TIMEOUT_SECS {
                if let Ok(()) = channel0.set_duty(0) {
                    display_off = true;
                }
            }
        }
        Timer::after(Duration::from_millis(100)).await
    }
}
