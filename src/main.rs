#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;

use alloc::string::ToString;
use alloc::vec;
use alloc::vec::Vec;
use core::alloc::{GlobalAlloc, Layout};
use core::cell::RefCell;
use core::fmt::Write;
use core::str::from_utf8_unchecked;

use display_interface_spi::SPIInterfaceNoCS;
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
use embedded_graphics::geometry::{Dimensions, Point, Size};
use embedded_graphics::mono_font::{MonoFont, MonoTextStyle};
use embedded_graphics::pixelcolor::{Rgb565, RgbColor};
use embedded_graphics::prelude::Primitive;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};
use embedded_graphics::text::{Alignment, Baseline, TextStyle, TextStyleBuilder};
use embedded_graphics::text::renderer::TextRenderer;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::i2c::I2c;
use embedded_iconoir::icons;
use embedded_iconoir::prelude::IconoirNewIcon;
use embedded_svc::io::asynch::BufRead;
use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use esp32_hal::{clock::ClockControl, embassy, IO, peripherals::Peripherals, prelude::*, psram};
use esp32_hal::{Rng, timer::TimerGroup};
use esp32_hal::clock::Clocks;
use esp32_hal::dma::{DmaDescriptor, DmaPriority};
use esp32_hal::gpio::{GpioPin, NO_PIN, Unknown};
use esp32_hal::i2c::I2C;
use esp32_hal::i2s::{DataFormat, I2s, Standard};
use esp32_hal::i2s::asynch::I2sWriteDmaAsync;
use esp32_hal::ledc::{channel, HighSpeed, LEDC, timer};
use esp32_hal::pdma::{Dma, I2s0DmaChannel};
use esp32_hal::peripherals::{I2C0, I2S0};
use esp32_hal::spi::master::{Instance, Spi};
use esp32_hal::spi::SpiMode;
use esp32_utils_crate::dummy_pin::DummyPin;
use esp32_utils_crate::fonts::CharacterStyles;
use esp32_utils_crate::sdcard::SdcardManager;
use esp32_utils_crate::touch_mapper::TouchPosMapper;
use esp32_utils_crate::tsc2007::{Tsc2007, TSC2007_ADDR};
use esp32_utils_crate::tsc2007;
use esp_backtrace;
use esp_println::println;
use esp_wifi::{EspWifiInitFor, initialize};
use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState};
use heapless::String;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
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
use crate::graphics::{Button, GraphicUtils, ListItem, Progress, Theme};

mod graphics;

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
const STREAM_BUFFER_SIZE: usize = 1024 * 16;

const MAX_SAMPLES_PER_FRAME_BYTE: usize = MAX_SAMPLES_PER_FRAME * 2;

const I2S_BUFFER_SIZE: usize = 32000;

const META_DATA_TITLE_LEN_MAX: usize = 256;

const SILENCE_DELAY_FRAME: u64 = 32;

#[derive(Debug, Clone)]
struct MetaData {
    title: String<META_DATA_TITLE_LEN_MAX>,
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
    frames_send: u64,
    send_delay_sum: u64,
    frames_received: u64,
    receive_delay_sum: u64,
}

static SEND_STATS_DATA_SIGNAL: Signal<CriticalSectionRawMutex, StatsData> = Signal::new();
static RECEIVE_STATS_DATA_SIGNAL: Signal<CriticalSectionRawMutex, StatsData> = Signal::new();


const RADIO_STATION_URL_LEN_MAX: usize = 256;

#[derive(Clone, Debug)]
struct ControlData {
    url: String<RADIO_STATION_URL_LEN_MAX>,
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
}

impl StatusData {
    fn new_default() -> Self {
        StatusData {
            paused: true,
            url_index: 0,
            error: false,
            error_type: NoError,
        }
    }

    fn new_error(error_type: Error) -> Self {
        StatusData {
            paused: false,
            url_index: 0,
            error: true,
            error_type,
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
    url: String<RADIO_STATION_URL_LEN_MAX>,
    title: String<META_DATA_TITLE_LEN_MAX>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct RadioStationList {
    list: Vec<RadioStation>,
}

impl ListItem for RadioStation {
    fn get_text(&self) -> String<256> {
        String::from(self.title.as_str())
    }

    fn get_height(&self) -> u16 {
        self.get_font().character_size.height as u16
    }

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

impl Theme {
    fn new_dark_theme() -> Self {
        Theme {
            button_background_color: Rgb565::new(9, 37, 20),
            button_foreground_color: Rgb565::WHITE,
            screen_background_color: Rgb565::BLACK,
            text_color_primary: Rgb565::WHITE,
            highlight_color: Rgb565::new(15, 30, 15),
            error_color: Rgb565::RED,
        }
    }
    fn new_light_theme() -> Self {
        Theme {
            button_background_color: Rgb565::new(9, 37, 20),
            button_foreground_color: Rgb565::WHITE,
            screen_background_color: Rgb565::WHITE,
            text_color_primary: Rgb565::BLACK,
            highlight_color: Rgb565::new(24, 49, 24),
            error_color: Rgb565::RED,
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

async fn send_silence() {
    let empty_frame_data = FrameData::new();
    for x in 0..SILENCE_DELAY_FRAME {
        FRAME_CHANNEL.send(empty_frame_data).await;
    }
}

#[embassy_executor::task]
pub async fn handle_radio_stream(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>, default_url: &'static str) {
    let mut current_url: String<RADIO_STATION_URL_LEN_MAX> = String::from(default_url);
    let mut playback_paused = true;

    // let mut tls_read_buffer = [0; 4096];
    // let mut tls_write_buffer = [0; 4096];
    let mut rx_buffer = [0; 4096];

    let layout = Layout::array::<u8>(STREAM_BUFFER_SIZE).unwrap();
    let data_buffer_ptr = unsafe { ALLOCATOR.alloc(layout) };
    let data_buffer = unsafe { core::slice::from_raw_parts_mut(data_buffer_ptr, layout.size()) };

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
                current_url.push_str(control_data.url.as_str()).unwrap();
            }
            if control_data.play {
                playback_paused = false;
            }
            continue;
        }
        println!("connecting to {}", current_url);

        let url: String<RADIO_STATION_URL_LEN_MAX> = String::from(current_url.as_str());
        let mut request = http_client.request(GET, url.as_str()).await.unwrap();
        let headers = [("Icy-MetaData", "1")];
        request = request.headers(&headers);

        let mut icy_metaint = 0;
        let response_result = request.send(&mut rx_buffer).await;
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
                let mut frames_send = 0u64;

                let mut meta_data = MetaData {
                    title: String::from(""),
                    bitrate: 0,
                    channels: 0,
                    sample_rate: 0,
                };
                let mut frame_data = FrameData::new();
                let mut send_delay_sum = 0;
                let mut stats_data = StatsData {
                    frames_send: 0,
                    send_delay_sum: 0,
                    frames_received: 0,
                    receive_delay_sum: 0,
                };
                let mut stats_data_start = Instant::now();

                'outer: loop {
                    if CONTROL_DATA_SIGNAL.signaled() {
                        let control_data = CONTROL_DATA_SIGNAL.wait().await;
                        println!("outer loop {:?}", control_data);
                        if control_data.change_url {
                            current_url.clear();
                            current_url.push_str(control_data.url.as_str()).unwrap();

                            // send silence before change
                            send_silence().await;
                            break;
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
                                        let meta_data_str = unsafe {
                                            if current_meta.len() > META_DATA_TITLE_LEN_MAX {
                                                from_utf8_unchecked(&current_meta.as_slice()[..META_DATA_TITLE_LEN_MAX])
                                            } else { from_utf8_unchecked(&current_meta.as_slice()) }
                                        };

                                        meta_data.title = String::from(meta_data_str);
                                        META_DATA_SIGNAL.signal(meta_data.clone());
                                        // println!("{}", meta_data_str);
                                        in_meta = false;
                                        global_index = 0;
                                    }
                                    continue;
                                } else {
                                    data_buffer[data_buffer_index] = *b;
                                    data_buffer_index = data_buffer_index + 1;
                                    if data_buffer_index == STREAM_BUFFER_SIZE {
                                        // cant handle 320 so just shut up
                                        if meta_data.bitrate != 0 && meta_data.bitrate > 256 {
                                            println!("mp3 decoder error");
                                            playback_paused = true;

                                            STATUS_DATA_SIGNAL.signal(StatusData::new_error(Error::DecoderError));

                                            // send silence before pause
                                            send_silence().await;
                                            break 'outer;
                                        } else {
                                            let mut consumed = 0;
                                            while let Some((frame, bytes_consumed)) = decoder.next(&data_buffer[consumed..], &mut frame_data.data) {
                                                consumed += bytes_consumed;
                                                if let Frame::Audio(audio) = frame {
                                                    if meta_data.is_incomplete() {
                                                        meta_data.channels = audio.channels();
                                                        meta_data.sample_rate = audio.sample_rate();
                                                        meta_data.bitrate = audio.bitrate();
                                                        META_DATA_SIGNAL.signal(meta_data.clone());
                                                    }
                                                    if frame_id > SILENCE_DELAY_FRAME {
                                                        frame_data.id = frames_send;
                                                        match FRAME_CHANNEL.try_send(frame_data) {
                                                            Err(e) => {
                                                                let send_delay_start = Instant::now();
                                                                FRAME_CHANNEL.send(frame_data).await;
                                                                send_delay_sum += send_delay_start.elapsed().as_micros();
                                                            }
                                                            _ => {}
                                                        }
                                                        frames_send += 1;
                                                    }
                                                    frame_id += 1;
                                                    break;
                                                }
                                            }
                                            // move forward
                                            data_buffer_index -= consumed;
                                            for i in 0..data_buffer_index {
                                                data_buffer[i] = data_buffer[i + consumed];
                                            }
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
                    if stats_data_start.elapsed().as_secs() > 5 {
                        stats_data.frames_send = frames_send;
                        stats_data.send_delay_sum = send_delay_sum;
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
pub async fn handle_frame_stream(i2s: I2s<'static, I2S0, I2s0DmaChannel>, bclk_pin: GpioPin<Unknown, 26>, ws_pin: GpioPin<Unknown, 25>,
                                 dout_pin: GpioPin<Unknown, 12>) {
    let i2s_tx = i2s
        .i2s_tx
        .with_bclk(bclk_pin)
        .with_ws(ws_pin)
        .with_dout(dout_pin)
        .build();

    let tx_buffer = make_static!([0u8; I2S_BUFFER_SIZE]);

    let mut transaction = i2s_tx.write_dma_circular_async(tx_buffer).unwrap();
    let mut total_bytes: u64 = 0;
    let mut receive_delay_sum: u64 = 0;
    let mut frame_id: u64 = 0;
    let mut stats_data = StatsData {
        frames_send: 0,
        send_delay_sum: 0,
        frames_received: 0,
        receive_delay_sum: 0,
    };
    let mut stats_start_init = false;
    let mut stats_data_start = Instant::now();
    let mut frames_received: u64 = 0;

    loop {
        let mut frame_data_option: Option<FrameData> = None;
        if let Ok(frame_data) = FRAME_CHANNEL.try_receive() {
            frame_data_option = Some(frame_data);
        } else {
            let receive_delay_start = Instant::now();
            frame_data_option = Some(FRAME_CHANNEL.receive().await);
            receive_delay_sum += receive_delay_start.elapsed().as_micros();
        }

        let frame_data = frame_data_option.unwrap();
        frames_received += 1;
        // new station
        if frame_data.id < frame_id || !stats_start_init {
            stats_data.frames_send = 0;
            stats_data.send_delay_sum = 0;
            stats_data.frames_received = 0;
            stats_data.receive_delay_sum = 0;
            frames_received = 0;
            stats_data_start = Instant::now();
            stats_start_init = true;
        }
        frame_id = frame_data.id;
        let frame_data_part = frame_data.data;
        let frame_data_part_u8 =
            unsafe { core::slice::from_raw_parts(&frame_data_part as *const _ as *const u8, frame_data_part.len() * 2) };
        let mut written_bytes = 0;
        while written_bytes != frame_data_part_u8.len() {
            match transaction.push(&frame_data_part_u8[written_bytes..]).await {
                Ok(written) => {
                    written_bytes += written;
                    total_bytes += written_bytes as u64;
                }
                Err(e) => { println!("transaction.push error = {:?}", e) }
            }
        }
        if stats_data_start.elapsed().as_secs() > 5 {
            stats_data.frames_received = frames_received;
            stats_data.receive_delay_sum = receive_delay_sum;
            RECEIVE_STATS_DATA_SIGNAL.signal(stats_data.clone());
            stats_data_start = Instant::now();
        }
    }
}

#[embassy_executor::task]
async fn handle_tp_touch_tsc2007(i2c: asynch::i2c::I2cDevice<'static, CriticalSectionRawMutex, I2C<'static, I2C0>>, tp_irq_pin: GpioPin<Unknown, 32>,
                                 orientation: Orientation, width: u16, height: u16) {
    let mut tsc2007 = Tsc2007::new(i2c);

    let mut handle_touch: bool = false;
    let mut tp_irq_input = tp_irq_pin.into_pull_down_input();

    let pos_mapper = TouchPosMapper::new(240, 320, (tsc2007::TS_MINX, tsc2007::TS_MAXX), (tsc2007::TS_MINY, tsc2007::TS_MAXY));

    loop {
        tp_irq_input.wait_for_low().await.unwrap();
        if let Ok(point) = tsc2007.touch().await {
            let x = point.0;
            let y = point.1;
            let z = point.2;
            if z > tsc2007::TS_MIN_PRESSURE {
                if !handle_touch {
                    handle_touch = true;
                    // println!("{:?}", point);

                    let (x_scaled, y_scaled) = match orientation {
                        Orientation::Portrait => pos_mapper.map_touch_pos(x, y, width, height, 0),
                        Orientation::Landscape => pos_mapper.map_touch_pos(x, y, width, height, 1),
                        Orientation::PortraitFlipped => pos_mapper.map_touch_pos(x, y, width, height, 2),
                        Orientation::LandscapeFlipped => pos_mapper.map_touch_pos(x, y, width, height, 3),
                    };

                    TOUCH_DATA_SIGNAL.signal(TouchData::new(x_scaled, y_scaled, z));
                }
            } else {
                handle_touch = false;
            }
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

#[main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();

    psram::init_psram(peripherals.PSRAM);
    init_psram_heap();

    let system = peripherals.SYSTEM.split();

    let clocks = singleton!(
        ClockControl::max(system.clock_control).freeze(),
        Clocks
    );
    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0);

    let timer_group1 = TimerGroup::new(peripherals.TIMG1, &clocks);
    let mut rng = Rng::new(peripherals.RNG);
    let _init = initialize(
        EspWifiInitFor::Wifi,
        timer_group1.timer0,
        rng,
        system.radio_clock_control,
        &clocks,
    ).unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    // backlight
    let bl_pin = io.pins.gpio4.into_push_pull_output();
    let mut ledc = LEDC::new(
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
            duty_pct: 30,
            pin_config: channel::config::PinConfig::PushPull,
        })
        .unwrap();

    // enable i2c_power
    let i2c_power = io.pins.gpio2;
    i2c_power.into_push_pull_output().set_high().unwrap();

    let i2c0 = I2C::new(
        peripherals.I2C0,
        io.pins.gpio22,
        io.pins.gpio20,
        100u32.kHz(),
        clocks,
    );

    let i2c0_bus = mutex::Mutex::<blocking_mutex::raw::CriticalSectionRawMutex, _>::new(i2c0);
    let i2c0_bus_static = make_static!(i2c0_bus);

    let mut i2c0_dev1 = asynch::i2c::I2cDevice::new(i2c0_bus_static);
    let has_tsc2007 = i2c0_dev1.read(TSC2007_ADDR, &mut [0]).await.is_ok();
    println!("has_tsc2007 = {}", has_tsc2007);

    let sclk = io.pins.gpio5;
    let miso = io.pins.gpio21;
    let mosi = io.pins.gpio19;
    let dc = io.pins.gpio33.into_push_pull_output();

    let display_cs = io.pins.gpio15.into_push_pull_output();
    let sd_cs = io.pins.gpio14.into_push_pull_output();

    let mut rst = DummyPin;

    let spi2 = Spi::new(peripherals.SPI2, 20u32.MHz(), SpiMode::Mode0, clocks)
        .with_pins(Some(sclk), Some(mosi), Some(miso), NO_PIN);

    let spi2_bus = blocking_mutex::Mutex::<blocking_mutex::raw::CriticalSectionRawMutex, _>::new(RefCell::new(spi2));

    let display_spi = blocking::spi::SpiDevice::new(
        &spi2_bus,
        display_cs);
    let sd_spi = blocking::spi::SpiDevice::new(
        &spi2_bus,
        DummyPin);

    let spi_iface = SPIInterfaceNoCS::new(display_spi, dc);

    let mut delay = Delay;

    let display_orientation = Orientation::Landscape;
    let display_size = DisplaySize240x320;

    let mut display = Ili9341::new(
        spi_iface,
        &mut rst,
        &mut delay,
        display_orientation,
        display_size,
    ).unwrap();


    let theme = Theme::new_light_theme();
    let mut character_styles = CharacterStyles::new_with_color(theme.text_color_primary);
    character_styles.set_background_color(theme.screen_background_color);

    let display_width = display.width() as u32;
    let display_height = display.height() as u32;

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

    let sdcard = embedded_sdmmc::sdcard::SdCard::new(sd_spi, sd_cs, Delay);
    let mut sdcard_manager = SdcardManager::new(sdcard);

    let mut root_file_list = Vec::new();
    if let Ok(()) = sdcard_manager.open_root_dir() {
        match sdcard_manager.get_root_dir_entries(&mut root_file_list) {
            Ok(()) => {}
            Err(e) => println!("{:?}", e)
        }
    }
    let mut station_list: Vec<RadioStation> = Vec::new();

    // for file in &root_file_list {
    //     if file.name.to_string() == "RADIO.JSO" {
    //         let mut buffer_vec = vec![0; file.size as usize];
    //         let load_result = sdcard_manager.load_root_dir_file_into_buffer("RADIO.JSO", &mut buffer_vec);
    //         if load_result.is_ok() {
    //             let mut sd_station_list = serde_json_core::from_slice::<RadioStationList>(buffer_vec.as_slice()).unwrap().0;
    //             station_list.append(&mut sd_station_list.list);
    //         }
    //     }
    // }

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

    display.clear_screen(theme.screen_background_color).unwrap();

    let dma = Dma::new(system.dma);
    let dma_channel = dma.i2s0channel;

    // from dma_buffers macro
    let tx_descriptors = make_static!([DmaDescriptor::EMPTY; (I2S_BUFFER_SIZE + 4091) / 4092]);
    let rx_descriptors = make_static!([DmaDescriptor::EMPTY; (I2S_BUFFER_SIZE + 4091) / 4092]);

    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data16Channel16,
        44100u32.Hz(),
        dma_channel.configure(
            false,
            tx_descriptors,
            rx_descriptors,
            DmaPriority::Priority0,
        ),
        clocks,
    );

    if station_list.len() == 0 {
        station_list.push(RadioStation::new("FM4", "http://orf-live.ors-shoutcast.at/fm4-q2a"));
        station_list.push(RadioStation::new("MetalRock.FM", "http://cheetah.streemlion.com:2160/stream"));
        station_list.push(RadioStation::new("Terry Callaghan's Classic Alternative Channel", "http://s16.myradiostream.com:7304"));
        station_list.push(RadioStation::new("OE3", "http://orf-live.ors-shoutcast.at/oe3-q2a"));
        station_list.push(RadioStation::new("OE3", "http://orf-live.ors-shoutcast.at/oe3-q2a"));
    }

    let mut current_screen = 0;

    let mut select_list = graphics::List::new(&station_list, Point::new(10, 10),
                                              Size::new(display_width - 10, (display_height - GraphicUtils::get_button_size().height)),
                                              &theme);
    select_list.draw(&mut display).unwrap();
    display_list_navigation(&mut display, display_width, display_height, &theme).unwrap();

    let default_url: &mut String<RADIO_STATION_URL_LEN_MAX> = make_static!(String::from(station_list.first().unwrap().url.as_str()));

    spawner.must_spawn(handle_radio_stream(stack, default_url));
    spawner.must_spawn(handle_frame_stream(i2s, io.pins.gpio26, io.pins.gpio25, io.pins.gpio12));
    if has_tsc2007 {
        spawner.must_spawn(handle_tp_touch_tsc2007(i2c0_dev1, io.pins.gpio32, display_orientation, display_width as u16, display_height as u16));
    }

    // let mut file_written = 0;
    // let mut file_done = false;
    // let mut time = Instant::now();
    let mut current_sample_rate = 44100u32;
    let clear_style = PrimitiveStyleBuilder::new()
        .fill_color(theme.screen_background_color)
        .build();

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
    loop {
        if META_DATA_SIGNAL.signaled() {
            meta_data = META_DATA_SIGNAL.wait().await;

            if current_screen == 1 {
                display_play_navigation(&mut display, display_width, display_height, &status, &theme).unwrap();

                // clear error
                GraphicUtils::display_text_with_background(&mut display, error_line_pos, error_character_style,
                                                           character_styles.default_text_style(),
                                                           GraphicUtils::get_text_with_ellipsis_from_str(display_width, " ", character_styles.medium_character_style().font).as_str(),
                                                           clear_style, display_width).unwrap();

                GraphicUtils::display_text_with_background(&mut display, title_line_pos, character_styles.large_character_style(),
                                                           character_styles.default_text_style(), station_list[status.url_index].title.as_str(),
                                                           clear_style, display_width).unwrap();

                let mut buf: String<META_DATA_TITLE_LEN_MAX> = String::new();

                let mut title = meta_data.title.strip_prefix("StreamTitle='").unwrap_or(meta_data.title.as_str());
                title = title.strip_suffix("';").unwrap_or(meta_data.title.as_str());

                let split_pos = title.find(" - ").unwrap_or(0);
                if split_pos != 0 {
                    let (artist, track) = title.split_at(split_pos);
                    GraphicUtils::display_text_with_background(&mut display, meta_first_line_pos, character_styles.medium_character_style(),
                                                               character_styles.default_text_style(),
                                                               GraphicUtils::get_text_with_ellipsis_from_str(display_width, artist, character_styles.medium_character_style().font).as_str(),
                                                               clear_style, display_width).unwrap();
                    GraphicUtils::display_text_with_background(&mut display, meta_second_line_pos, character_styles.medium_character_style(),
                                                               character_styles.default_text_style(),
                                                               GraphicUtils::get_text_with_ellipsis_from_str(display_width, track.strip_prefix(" - ").unwrap_or(track), character_styles.medium_character_style().font).as_str(),
                                                               clear_style, display_width).unwrap();
                } else {
                    GraphicUtils::display_text_with_background(&mut display, meta_first_line_pos, character_styles.medium_character_style(),
                                                               character_styles.default_text_style(),
                                                               GraphicUtils::get_text_with_ellipsis_from_str(display_width, title, character_styles.medium_character_style().font).as_str(),
                                                               clear_style, display_width).unwrap();
                }

                buf.clear();
                write!(buf, "{} / {} / {}", meta_data.sample_rate, meta_data.channels, meta_data.bitrate).unwrap();

                GraphicUtils::display_text_with_background(&mut display, format_line_pos, character_styles.medium_character_style(),
                                                           character_styles.default_text_style(),
                                                           buf.as_str(),
                                                           clear_style, display_width).unwrap();
            }
            if meta_data.sample_rate != 0 {
                if meta_data.sample_rate != current_sample_rate {
                    println!("change sample rate to {}", meta_data.sample_rate);
                    current_sample_rate = meta_data.sample_rate;
                    esp32_hal::i2s::private::update_config(Standard::Philips,
                                                           DataFormat::Data16Channel16,
                                                           meta_data.sample_rate.Hz(),
                                                           &clocks);
                }
            }
        }
        if SEND_STATS_DATA_SIGNAL.signaled() {
            let stats_data = SEND_STATS_DATA_SIGNAL.wait().await;
            if stats_data.frames_send != 0 {
                println!("send: {} {} {}", stats_data.frames_send, stats_data.send_delay_sum, stats_data.send_delay_sum / stats_data.frames_send);
            }
        }
        if RECEIVE_STATS_DATA_SIGNAL.signaled() {
            let stats_data = RECEIVE_STATS_DATA_SIGNAL.wait().await;
            if stats_data.frames_received != 0 {
                println!("receive: {} {} {}", stats_data.frames_received, stats_data.receive_delay_sum, stats_data.receive_delay_sum / stats_data.frames_received);
            }
        }

        if TOUCH_DATA_SIGNAL.signaled() {
            let touch_data = TOUCH_DATA_SIGNAL.wait().await;
            let touch_point = Point::new(touch_data.x as i32, touch_data.y as i32);

            if icon_left_area.contains(touch_point) {
                if current_screen == 0 {
                    select_list.scroll_up(&mut display).unwrap();
                } else if current_screen == 1 {
                    println!("control change_url");
                    let url_index_new = (status.url_index + 1) % station_list.len();
                    status.set_playing(url_index_new);
                    select_list.set_selected_index(url_index_new);
                    CONTROL_DATA_SIGNAL.signal(ControlData::new_change_url(&station_list[url_index_new].url));
                }
            }
            if icon_right_area.contains(touch_point) {
                if current_screen == 0 {
                    select_list.scroll_down(&mut display).unwrap();
                } else if current_screen == 1 {
                    println!("control play_pause");
                    status.toggle_playing();
                    if status.paused {
                        CONTROL_DATA_SIGNAL.signal(ControlData::new_pause());
                    } else {
                        CONTROL_DATA_SIGNAL.signal(ControlData::new_play(&station_list[status.url_index].url));
                    }
                    display_play_navigation(&mut display, display_width, display_height, &status, &theme).unwrap();
                }
            }
            if icon_middle_area.contains(touch_point) {
                if current_screen == 0 {
                    current_screen = 1;
                    display.clear_screen(theme.screen_background_color).unwrap();

                    let url_index_new = select_list.get_selected_index();
                    status.set_playing(url_index_new);
                    CONTROL_DATA_SIGNAL.signal(ControlData::new_change_url(&station_list[url_index_new].url));

                    display_play_navigation(&mut display, display_width, display_height, &status, &theme).unwrap();
                } else if current_screen == 1 {
                    current_screen = 0;
                    display.clear_screen(theme.screen_background_color).unwrap();

                    select_list.draw(&mut display).unwrap();
                    display_list_navigation(&mut display, display_width, display_height, &theme).unwrap();
                }
            }
            if select_list.get_bounding_box().contains(touch_point) {
                if let Ok(selected_index) = select_list.select_at_pos(&mut display, touch_point) {
                    current_screen = 1;
                    display.clear_screen(theme.screen_background_color).unwrap();

                    status.set_playing(selected_index);
                    CONTROL_DATA_SIGNAL.signal(ControlData::new_change_url(&station_list[selected_index].url));

                    display_play_navigation(&mut display, display_width, display_height, &status, &theme).unwrap();
                }
            }
        }
        if STATUS_DATA_SIGNAL.signaled() {
            let error_status = STATUS_DATA_SIGNAL.wait().await;
            if error_status.error {
                status.set_paused();
                if current_screen == 1 {
                    display_play_navigation(&mut display, display_width, display_height, &status, &theme).unwrap();

                    GraphicUtils::display_text_with_background(&mut display, error_line_pos, error_character_style,
                                                               character_styles.default_text_style(),
                                                               GraphicUtils::get_text_with_ellipsis_from_str(display_width,
                                                                                                             get_error_description(error_status.error_type),
                                                                                                             character_styles.medium_character_style().font).as_str(),
                                                               clear_style, display_width).unwrap();
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

        Timer::after(Duration::from_millis(100)).await
    }
}
