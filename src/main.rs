#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

extern crate alloc;

use alloc::vec;
use alloc::vec::Vec;
use core::alloc::GlobalAlloc;
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
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_graphics::pixelcolor::{Rgb565, RgbColor};
use embedded_hal_async::i2c::I2c;
use embedded_svc::io::asynch::BufRead;
use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use esp32_hal::{clock::ClockControl, embassy, IO, peripherals::Peripherals, prelude::*, psram};
use esp32_hal::{Rng, timer::TimerGroup};
use esp32_hal::clock::Clocks;
use esp32_hal::gpio::NO_PIN;
use esp32_hal::i2c::I2C;
use esp32_hal::ledc::{channel, HighSpeed, LEDC, timer};
use esp32_hal::spi::master::{Instance, Spi};
use esp32_hal::spi::SpiMode;
use esp_backtrace;
use esp_println::{print, println};
use esp_wifi::{EspWifiInitFor, initialize};
use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiStaDevice, WifiState};
use heapless::String;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use reqwless::client::{HttpClient, TlsConfig, TlsVerify};
use reqwless::request::Method::GET;
use reqwless::request::RequestBuilder;
use reqwless::response::Status;
use rmp3::{Frame, MAX_SAMPLES_PER_FRAME, RawDecoder, Sample};
use rust_utils::dummy_pin::DummyPin;
use static_cell::make_static;
use static_cell::StaticCell;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_psram_heap() {
    unsafe {
        ALLOCATOR.init(psram::PSRAM_VADDR_START as *mut u8, psram::PSRAM_BYTES);
    }
}

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

static STREAM_CHANNEL: Channel<CriticalSectionRawMutex, [u8; 8192], 4> = Channel::new();

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
pub async fn test_stream(stack: &'static Stack<WifiDevice<'static, WifiStaDevice>>) {
    let mut url: String<1024> = String::new();
    // write!(url, "http://orf-live.ors-shoutcast.at/fm4-q2a").unwrap();
    // write!(url, "http://192.168.1.15:8080/stream").unwrap();
    write!(url, "http://uk2.internet-radio.com:8171/stream").unwrap();

    let mut tls_read_buffer = [0; 4096];
    let mut tls_write_buffer = [0; 4096];
    let mut rx_buffer = [0; 2048];

    let client_state = TcpClientState::<1, 1024, 1024>::new();
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
            let mut body_reader = response.body().reader();
            let mut data_buffer = [0u8; 8192];
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
                                    println!("{}", unsafe { from_utf8_unchecked(current_meta.as_slice()) });
                                    in_meta = false;
                                    global_index = 0;
                                }
                                continue;
                            } else {
                                data_buffer[data_buffer_index] = *b;
                                data_buffer_index = data_buffer_index + 1;
                                if data_buffer_index == 8192 {
                                    // println!("{}", time.elapsed().as_micros());
                                    println!("send buffer");
                                    STREAM_CHANNEL.send(data_buffer.clone()).await;
                                    time = Instant::now();
                                    data_buffer_index = 0;
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

    let mut rst = DummyPin;

    let spi2 = Spi::new(peripherals.SPI2, 26u32.MHz(), SpiMode::Mode0, clocks)
        .with_pins(Some(sclk), Some(mosi), Some(miso), NO_PIN);

    let spi2_bus = blocking_mutex::Mutex::<blocking_mutex::raw::CriticalSectionRawMutex, _>::new(RefCell::new(spi2));

    let display_spi = blocking::spi::SpiDevice::new(
        &spi2_bus,
        display_cs);

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

    let background_color_default = Rgb565::BLACK;

    display.clear_screen(background_color_default).unwrap();


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

    spawner.must_spawn(test_stream(stack));

    let mut decoder = RawDecoder::new();
    let mut buf = [Sample::default(); MAX_SAMPLES_PER_FRAME];

    loop {
        let stream_data: [u8; 8192] = STREAM_CHANNEL.receive().await;
        let mut time = Instant::now();
        println!("receive buffer");
        let mut index = 0;

        while let Some((frame, bytes_consumed)) = decoder.next(&stream_data[index..], &mut buf) {
            if let Frame::Audio(audio) = frame {
                // process audio frame here!
                // println!("audio frame = {} {} {} {} {}", audio.bitrate(), audio.channels(), audio.mpeg_layer(), audio.sample_rate(), buf[0]);
                // println!("audio frame = {} {} {} {} {} {} {} {}", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
                // println!("{}", audio.samples().len());
            }
            index = index + bytes_consumed;
        }
        println!("ellapsed = {}", time.elapsed().as_millis());
    }
}
