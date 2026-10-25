//! C3 rtl_tcp 节拍流：AP 模式自建热点 → TCP 服务端（端口 1234）→
//! 突发采集 + IQ8 打包按 2 KB 分块写入，TCP 背压自然节拍。
//!
//! 协议（rtl_tcp 标准）：
//! - 客户端连接后发若干 5 字节命令（1 字节命令码 + 4 字节大端参数），
//!   本机实现 0x02 SET_FREQ（真实调谐），其余消费忽略。
//! - 服务端先发 12 字节头："RTL0" + 调谐器类型(u32 BE) + 增益档数(u32 BE)，
//!   然后持续输出 8 bit 无符号 I/Q 交织流。
//!
//! PC 侧：连接板卡热点 "esp-sdr"（开放网络），SDR++/客户端指向 192.168.2.1:1234，
//! 采样率设 250 kS/s。

#![no_std]
#![no_main]

use core::net::{Ipv4Addr, SocketAddrV4};

use embassy_executor::Spawner;
use embassy_net::{
    IpListenEndpoint,
    Ipv4Cidr,
    Runner,
    Stack,
    StackResources,
    StaticConfigV4,
    tcp::TcpSocket,
};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
use esp_alloc as _;
use esp_backtrace as _;
use esp_println as _;
use esp_radio::wifi::{
    Config,
    ControllerConfig,
    Interface,
    WifiController,
    ap::AccessPointConfig,
};
use esp_sdr_mmio::{CaptureRate, DumpEngine};

esp_bootloader_esp_idf::esp_app_desc!();

const AP_SSID: &str = "esp-sdr";
const GATEWAY_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 2, 1);
const TCP_PORT: u16 = 1234;
const CAPTURE_WORD_COUNT: usize = 16380;

/// IQ8 打包暂存（2048 样本对 = 4096 字节，对齐 TCP 写块）。
/// 仅采集任务访问（单写单读、无并发），用 UnsafeCell 显式声明内部可变性。
struct StagingBuffer(core::cell::UnsafeCell<[u8; 4096]>);
unsafe impl Sync for StagingBuffer {}
static IQ8_STAGING: StagingBuffer = StagingBuffer(core::cell::UnsafeCell::new([0; 4096]));

/// rtl_tcp 客户端命令码（消费用；仅 0x02 SET_FREQ 真实生效）。
const CMD_SET_FREQ: u8 = 0x02;

/// IQ8 打包：按上游 pack_iq8 的位序（I=bits 9:0 → 字节偶位，Q=bits 19:10 → 字节奇位），
/// 10 bit 截取高 8 位。写入 staging 并返回字节数（2 字节/样本对）。
fn pack_iq8_into_staging(words: &[u32]) -> usize {
    let staging = unsafe { &mut *IQ8_STAGING.0.get() };
    let pairs = words.len();
    for (index, &word) in words.iter().enumerate() {
        staging[2 * index] = ((word >> 2) & 0xFF) as u8;
        staging[2 * index + 1] = ((word >> 12) & 0xFF) as u8;
    }
    2 * pairs
}

/// 解析 rtl_tcp 客户端命令（5 字节一组，大端参数）。
/// 返回需要应用的新频点（MHz），无则 None。
fn parse_commands(buffer: &[u8]) -> Option<u32> {
    let mut new_frequency_mhz = None;
    for chunk in buffer.chunks_exact(5) {
        if chunk[0] == CMD_SET_FREQ {
            let hz = u32::from_be_bytes([chunk[1], chunk[2], chunk[3], chunk[4]]);
            new_frequency_mhz = Some(hz / 1_000_000);
        }
    }
    new_frequency_mhz
}

/// 采集 + 打包 + TCP 写入一整个突发（16 KB IQ8，按 staging 分块）。
/// TCP write_all 的背压即节拍器。
///
/// 注意：AP 模式下接收增益由 PHY 的 AGC 管理（AP 上行需要它），
/// 不做手动增益锁；dump engine 采集与 AP 收发共用同一接收链。
async fn stream_one_burst(
    dump_engine: &mut DumpEngine,
    socket: &mut TcpSocket<'_>,
) -> Result<(), StreamError> {
    let capture_result = dump_engine.capture_burst(CAPTURE_WORD_COUNT, CaptureRate::Msps80);
    let sample_words = match capture_result {
        Ok((words, _elapsed_us)) => words,
        Err(error) => return Err(StreamError::Capture(error)),
    };

    // 按 staging 容量分块打包 + 写 TCP（每块 2048 样本对 = 4096 字节）
    for chunk in sample_words.chunks(2048) {
        let packed_len = pack_iq8_into_staging(chunk);
        let staging = unsafe { &*IQ8_STAGING.0.get() };
        socket
            .write_all(&staging[..packed_len])
            .await
            .map_err(StreamError::Tcp)?;
    }
    socket.flush().await.map_err(StreamError::Tcp)?;
    Ok(())
}

#[derive(Debug)]
enum StreamError {
    Capture(esp_sdr_mmio::CaptureError),
    Tcp(embassy_net::tcp::Error),
}

/// TCP 服务任务：接受连接 → 协议头 → 消费命令 → 持续节拍流。
#[embassy_executor::task]
async fn tcp_server_task(stack: Stack<'static>) {
    stack.wait_config_up().await;
    defmt::info!("rtl_tcp listening on 192.168.2.1:{}", TCP_PORT as u32);

    // TCP 发送缓冲 8 KB 对 ~330 kB/s 的 WiFi 链路足够；embassy-net 内部分块搬运。
    static TCP_TX_CELL: static_cell::StaticCell<[u8; 8192]> = static_cell::StaticCell::new();
    static TCP_RX_CELL: static_cell::StaticCell<[u8; 1024]> = static_cell::StaticCell::new();

    let tcp_tx_buffer: &'static mut [u8] = TCP_TX_CELL.init([0; 8192]);
    let tcp_rx_buffer: &'static mut [u8] = TCP_RX_CELL.init([0; 1024]);

    loop {
        let mut socket = TcpSocket::new(stack, tcp_rx_buffer, tcp_tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(10)));

        defmt::info!("waiting for rtl_tcp client");
        if let Err(_error) = socket
            .accept(IpListenEndpoint {
                addr: None,
                port: TCP_PORT,
            })
            .await
        {
            defmt::error!("accept failed");
            Timer::after(Duration::from_millis(1_000)).await;
            continue;
        }
        defmt::info!("client connected");

        // 协议头："RTL0" + 调谐器类型 1(E4000) + 增益档数 0
        let header: [u8; 12] = [
            b'R', b'T', b'L', b'0', 0, 0, 0, 1, 0, 0, 0, 0,
        ];
        if let Err(_error) = socket.write_all(&header).await {
            defmt::error!("header write failed");
            continue;
        }

        // 消费客户端命令（连接即成串到达；一次读尽缓冲即可）。
        // C3 限制：SET_FREQ 仅记录——AP 模式下非标准频点调谐会停发信标，
        // 频点固定为信道 1（2412 MHz）；真调谐留待后续检查点。
        let mut command_buffer = [0u8; 128];
        if let Ok(received) = socket.read(&mut command_buffer).await {
            if let Some(mhz) = parse_commands(&command_buffer[..received]) {
                defmt::info!("client requested {} mhz (ignored in C3)", mhz);
            }
        }

        let mut dump_engine = DumpEngine::new();
        let mut bursts: u32 = 0;
        let stream_result = loop {
            match stream_one_burst(&mut dump_engine, &mut socket).await {
                Ok(()) => {
                    bursts += 1;
                    if bursts % 100 == 0 {
                        defmt::info!("streamed {} bursts", bursts);
                    }
                    // 节拍微调：交付速率对齐 ADVERTISED_RATE_HZ
                    // 每突发 8190 对；目标 250 kS/s → 周期 ~32.8 ms；
                    // TCP 写耗时即为大部分节拍，这里只补差值。
                    Timer::after(Duration::from_millis(10)).await;
                }
                Err(error) => break error,
            }
        };
        let _ = stream_result;
        defmt::error!("stream ended after {} bursts", bursts);
        socket.close();
    }
}

/// DHCP 服务任务：给加入热点的 PC 分配 192.168.2.x。
#[embassy_executor::task]
async fn dhcp_task(stack: Stack<'static>) {
    use edge_dhcp::{
        io::{self, DEFAULT_SERVER_PORT},
        server::{Server, ServerOptions},
    };
    use edge_nal::UdpBind;
    use edge_nal_embassy::{Udp, UdpBuffers};

    let mut buf = [0u8; 1500];
    let mut gw_buf = [Ipv4Addr::UNSPECIFIED];
    let buffers = UdpBuffers::<1, 1024, 1024, 4>::new();
    let unbound_socket = Udp::new(stack, &buffers);
    let mut bound_socket = unbound_socket
        .bind(core::net::SocketAddr::V4(SocketAddrV4::new(
            Ipv4Addr::UNSPECIFIED,
            DEFAULT_SERVER_PORT,
        )))
        .await
        .unwrap();

    loop {
        let _ = io::server::run(
            &mut Server::<_, 64>::new_with_et(GATEWAY_IP),
            &ServerOptions::new(GATEWAY_IP, Some(&mut gw_buf)),
            &mut bound_socket,
            &mut buf,
        )
        .await;
        Timer::after(Duration::from_millis(500)).await;
    }
}

/// embassy-net 协议栈任务。
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await
}

#[esp_hal::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_sdr_apps::init_heap();

    let timer_group0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timer_group0.timer0, peripherals.FROM_CPU_INTR0);

    // 网络设备先于控制器创建（对齐 esp-radio AP 示例）
    let device = Interface::access_point();
    let mut controller = WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(Config::AccessPoint(
            AccessPointConfig::default().with_ssid(AP_SSID.try_into().unwrap()),
        )),
    )
    .unwrap();

    // 注意（C2/C3 教训综合）：
    // 1. AP 模式本身让接收链持续在线，无需混杂模式——实测混杂模式与 AP
    //    共用单射频会卡死网络栈（信标停发）。
    // 2. 不做手动增益锁/PHY 重调——AP 的信标发送依赖 TX 链，
    //    prepare_receive 的 phy_pbus_xpd_tx_off 会停发信标；
    //    接收增益由 PHY 的 AGC 管理。
    // 3. 不调 set_channel——AP 默认信道 1；实测 AP 启动后再调
    //    esp_wifi_set_channel 会让信标消失（待查证）。

    let net_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(GATEWAY_IP, 24),
        gateway: Some(GATEWAY_IP),
        dns_servers: Default::default(),
    });

    let rng = esp_hal::rng::Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    static RESOURCES_CELL: static_cell::StaticCell<StackResources<3>> =
        static_cell::StaticCell::new();
    let resources = RESOURCES_CELL.init(StackResources::<3>::new());
    let (stack, runner) = embassy_net::new(device, net_config, resources, seed);

    let gw = GATEWAY_IP.octets();
    defmt::info!(
        "rtltcp ready: ap ssid={}, gw={}.{}.{}.{}",
        AP_SSID,
        gw[0],
        gw[1],
        gw[2],
        gw[3]
    );

    spawner.spawn(net_task(runner).unwrap());
    spawner.spawn(dhcp_task(stack).unwrap());
    spawner.spawn(tcp_server_task(stack).unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
