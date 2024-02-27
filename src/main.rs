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
use embedded_graphics::geometry::Point;
use embedded_graphics::pixelcolor::{Rgb565, RgbColor};
use embedded_graphics::text::{Text, TextStyle};
use embedded_graphics::text::renderer::TextRenderer;
use embedded_hal_async::i2c::I2c;
use embedded_svc::io::asynch::BufRead;
use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use esp32_hal::{clock::ClockControl, dma_buffers, embassy, IO, peripherals::Peripherals, prelude::*, psram};
use esp32_hal::{Rng, timer::TimerGroup};
use esp32_hal::clock::Clocks;
use esp32_hal::dma::DmaPriority;
use esp32_hal::gpio::NO_PIN;
use esp32_hal::i2c::I2C;
use esp32_hal::i2s::asynch::I2sWriteDmaAsync;
use esp32_hal::i2s::{DataFormat, I2s, I2sWriteDma, Standard};
use esp32_hal::ledc::{channel, HighSpeed, LEDC, timer};
use esp32_hal::pdma::Dma;
use esp32_hal::spi::master::{Instance, Spi};
use esp32_hal::spi::SpiMode;
use esp_backtrace;
use esp_println::println;
use esp_wifi::{EspWifiInitFor, initialize};
use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState};
use heapless::String;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use reqwless::client::{HttpClient, TlsConfig, TlsVerify};
use reqwless::request::Method::GET;
use reqwless::request::RequestBuilder;
use reqwless::response::Status;
use rmp3::{MAX_SAMPLES_PER_FRAME, RawDecoder, Sample};
use rmp3::Frame;
use rust_utils::dummy_pin::DummyPin;
use static_cell::make_static;
use static_cell::StaticCell;

use crate::fonts::CharacterStyles;
use crate::sdcard::SdcardManager;

mod fonts;
mod sdcard;

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

// static STREAM_CHANNEL: Channel<CriticalSectionRawMutex, [u8; STREAM_BUFFER_SIZE], 4> = Channel::new();


const MAX_SAMPLES_PER_FRAME_BYTE: usize = MAX_SAMPLES_PER_FRAME * 2;

// in psram
const FRAME_BUFFER_SIZE: usize = MAX_SAMPLES_PER_FRAME_BYTE * 16;
const I2S_BUFFER_SIZE: usize = 8192;

const META_DATA_TITLE_LEN_MAX: usize = 256;

struct MetaData {
    title: String<META_DATA_TITLE_LEN_MAX>,
}

static META_DATA_SIGNAL: Signal<CriticalSectionRawMutex, MetaData> = Signal::new();

static FRAME_CHANNEL: Channel<CriticalSectionRawMutex, [i16; MAX_SAMPLES_PER_FRAME], 8> = Channel::new();

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
            println!("Starting wifi");
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

#[embassy_executor::task]
pub async fn handle_radio_stream(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    let mut url: String<1024> = String::new();
    write!(url, "http://orf-live.ors-shoutcast.at/fm4-q2a").unwrap();
    // write!(url, "http://uk2.internet-radio.com:8171/stream").unwrap();

    let mut tls_read_buffer = [0; 4096];
    let mut tls_write_buffer = [0; 4096];
    let mut rx_buffer = [0; 4096];

    let layout = Layout::array::<u8>(STREAM_BUFFER_SIZE).unwrap();
    let data_buffer_ptr = unsafe { ALLOCATOR.alloc(layout) };
    let data_buffer = unsafe { core::slice::from_raw_parts_mut(data_buffer_ptr, layout.size()) };

    let client_state = TcpClientState::<1, 4096, 4096>::new();
    let tcp_client = TcpClient::new(&stack, &client_state);
    let dns = DnsSocket::new(&stack);

    let tls_config = TlsConfig::new(123456789u64, &mut tls_read_buffer, &mut tls_write_buffer, TlsVerify::None);
    let mut http_client = HttpClient::new_with_tls(&tcp_client, &dns, tls_config);

    let mut request = http_client.request(GET, url.as_str()).await.unwrap();
    // request = request.content_type(ContentType::ApplicationOctetStream);
    let headers = [("Icy-MetaData", "1")];
    request = request.headers(&headers);

    let mut icy_metaint = 0;
    let response = request.send(&mut rx_buffer).await.unwrap();
    if response.status == Status::Ok {
        for i in response.headers() {
            let key = i.0;
            if key.starts_with("icy") {
                println!("{}:{:?}", i.0, unsafe { from_utf8_unchecked(i.1) });
                if key.eq_ignore_ascii_case("icy-metaint") {
                    let icy_metaint_str = unsafe { from_utf8_unchecked(i.1) };
                    icy_metaint = icy_metaint_str.parse().unwrap();
                    println!("icy_metaint = {}", icy_metaint);
                }
            }
        }

        if icy_metaint == 0 {
            println!("No icy stream");
        } else {
            let mut decoder = RawDecoder::new();
            let mut sample_buf = [Sample::default(); MAX_SAMPLES_PER_FRAME];
            let mut body_reader = response.body().reader();
            // let mut data_buffer = [0u8; STREAM_BUFFER_SIZE];
            let mut data_buffer_index = 0;
            let mut global_index = 0;
            let mut in_meta = false;
            let mut in_meta_data_index = 0;
            let mut meta_data_len = 0;
            let mut current_meta: Vec<u8> = vec![];
            let time = Instant::now();

            loop {
                match (body_reader.fill_buf().await) {
                    Ok(mut buf) => {
                        if buf.is_empty() {
                            break;
                        }
                        let mut time = Instant::now();

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
                                current_meta.push(*b);
                                in_meta_data_index = in_meta_data_index + 1;
                                if in_meta_data_index == meta_data_len {
                                    let meta_data_str = unsafe {
                                        if current_meta.len() > META_DATA_TITLE_LEN_MAX {
                                            from_utf8_unchecked(&current_meta.as_slice()[..META_DATA_TITLE_LEN_MAX])
                                        } else { from_utf8_unchecked(&current_meta.as_slice()) }
                                    };
                                    let meta_data = MetaData {
                                        title: String::from(meta_data_str)
                                    };
                                    META_DATA_SIGNAL.signal(meta_data);
                                    // println!("{}", meta_data_str);
                                    in_meta = false;
                                    global_index = 0;
                                }
                                continue;
                            } else {
                                data_buffer[data_buffer_index] = *b;
                                data_buffer_index = data_buffer_index + 1;
                                if data_buffer_index == STREAM_BUFFER_SIZE {
                                    let mut consumed = 0;

                                    while let Some((frame, bytes_consumed)) = decoder.next(&data_buffer[consumed..], &mut sample_buf) {
                                        consumed += bytes_consumed;
                                        if let Frame::Audio(audio) = frame {
                                            // let mut sample_buf_copy = [Sample::default(); MAX_SAMPLES_PER_FRAME];
                                            // sample_buf_copy.copy_from_slice(audio.samples());
                                            // let data =
                                            //     unsafe { core::slice::from_raw_parts(audio.samples() as *const _ as *const u8, audio.samples().len() * 2) };
                                            // let mut sample_buf_u8 = [0u8; MAX_SAMPLES_PER_FRAME_BYTE];
                                            // sample_buf_u8.copy_from_slice(data);
                                            // let mut sample_count = 0;
                                            // for i in 0..data.len() {
                                            //     sample_buf_u8[i] = data[i];
                                            //     sample_count += 1;
                                            // }
                                            // println!("{} {} {}", sample_buf[0], sample_buf[1], sample_buf[2]);
                                            FRAME_CHANNEL.send(sample_buf).await;
                                            break;
                                        }
                                    }
                                    data_buffer_index -= consumed;
                                    {
                                        for i in 0..data_buffer_index {
                                            data_buffer[i] = data_buffer[i + consumed];
                                        }
                                    }
                                }
                                global_index = global_index + 1;
                            }
                        }
                        // println!("{}", time.elapsed().as_millis());

                        body_reader.consume(buf_len);
                    }
                    Err(e) => {
                        println!("{:?}", e);
                        break;
                    }
                }
            }
        }
    }
}

fn display_text<D, S>(display: &mut D, pos: Point, character_style: S,
                      text_style: TextStyle, text: &str) where D: DrawTarget<Color=Rgb565>, S: TextRenderer<Color=Rgb565> {
    let _ = Text::with_text_style(
        text,
        pos,
        character_style,
        text_style,
    )
        .draw(display);
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
    let mut i2c0_dev2 = asynch::i2c::I2cDevice::new(i2c0_bus_static);

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

    let mut character_styles = CharacterStyles::new();

    let background_color_default = Rgb565::BLACK;
    character_styles.set_background_color(background_color_default);

    display.clear_screen(background_color_default).unwrap();

    let sdcard = embedded_sdmmc::sdcard::SdCard::new(sd_spi, sd_cs, Delay);
    let mut sdcard_manager = SdcardManager::new(sdcard);

    let mut root_file_list = Vec::new();
    if let Ok(()) = sdcard_manager.open_root_dir() {
        match sdcard_manager.get_root_dir_entries(&mut root_file_list) {
            Ok(()) => {}
            Err(e) => println!("{:?}", e)
        }
        // if sdcard_manager.get_root_dir_entries(&mut root_file_list).is_err() {
        //     root_file_list.clear();
        // }
    }
    println!("root dir files size = {}", root_file_list.len());


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

    spawner.must_spawn(connection(controller));
    spawner.must_spawn(net_task(stack));

    println!("Connecting to Wifi...");
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    println!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    spawner.must_spawn(handle_radio_stream(stack));

    let mut str_buf: String<128> = String::new();

    let dma = Dma::new(system.dma);
    let dma_channel = dma.i2s0channel;

    let (tx_buffer, mut tx_descriptors, _, mut rx_descriptors) = dma_buffers!(8192, 0);

    // let layout = Layout::array::<u8>(FRAME_BUFFER_SIZE).unwrap();
    // let frame_buffer_ptr = unsafe { ALLOCATOR.alloc(layout) };
    // let frame_buffer = unsafe { core::slice::from_raw_parts_mut(frame_buffer_ptr, layout.size()) };

    let i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data16Channel16,
        44100u32.Hz(),
        2,
        dma_channel.configure(
            false,
            &mut tx_descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ),
        &clocks,
    );

    let i2s_tx = i2s
        .i2s_tx
        .with_bclk(io.pins.gpio26)
        .with_ws(io.pins.gpio25)
        .with_dout(io.pins.gpio12)
        .build();

    let mut transaction = i2s_tx.write_dma_circular_async(tx_buffer).unwrap();

    let mut i2s_buffer_index = 0;
    let mut i2s_buffer_written = 0;
    let time = Instant::now();
    let mut last_dt = 0;
    let mut frame_buffer_index = 0;

    let file = sdcard_manager.open_file_in_root_dir_for_writing("B.WAV").unwrap();

    let mut file_written = 0;
    let mut file_done = false;
    loop {
        if META_DATA_SIGNAL.signaled() {
            let meta_data = META_DATA_SIGNAL.wait().await;
            let title = meta_data.title.as_str();
            if title.contains("StreamTitle=") {
                let title_value = title.split_at("StreamTitle=".len()).1;
                display_text(&mut display, Point::new(0, 0), character_styles.default_character_style(),
                             character_styles.default_text_style(), title_value);
            }
        }
        let frame_data = FRAME_CHANNEL.receive().await;
        let frame_data_u8 =
            unsafe { core::slice::from_raw_parts(&frame_data as *const _ as *const u8, frame_data.len() * 2) };
        // frame_buffer[frame_buffer_index..(frame_buffer_index + MAX_SAMPLES_PER_FRAME_BYTE).min(FRAME_BUFFER_SIZE)].copy_from_slice(frame_data_u8);
        // frame_buffer_index += MAX_SAMPLES_PER_FRAME_BYTE;
        // if frame_buffer_index >= FRAME_BUFFER_SIZE {
        if !file_done {
            if file_written < 2000 {
                sdcard_manager.write_file_in_root_dir_from_buffer(file, frame_data_u8).unwrap();
                file_written += 1;
                // println!("wrote file part {}", file_written);
                // while i2s_buffer_index < FRAME_BUFFER_SIZE {
                //     match transaction.push(frame_data_u8).await {
                //         Ok(written) => {
                //             // println!("transaction.push written {}", written);
                //             // i2s_buffer_index += written;
                //             // i2s_buffer_written += written;
                //         }
                //         Err(e) => { println!("transaction.push error ={:?}", e) }
                //     }
                // }
            } else {
                sdcard_manager.close_file(file).unwrap();
                println!("close file");
                sdcard_manager.close_root_dir().unwrap();
                println!("close dir");
                file_done = true;
            }
        }
        // frame_buffer_index = 0;
        // }

        // i2s_buffer_index = 0;
        // let ms = time.elapsed().as_millis();
        // let s = (ms - last_dt);
        // if s >= 1000 {
        //     // println!("Length: {} : {}", s / 1000, i2s_buffer_written);
        //     last_dt = ms;
        //     i2s_buffer_written = 0;
        // }
    }
}
