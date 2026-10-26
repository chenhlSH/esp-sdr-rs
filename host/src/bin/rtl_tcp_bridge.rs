//! C3 rtl_tcp 桥：USB 串口收 capburst 帧 → 本地 rtl_tcp 服务端。
//!
//! 板卡跑 capburst（USB-Serial/JTAG 流），本程序在 PC 上把该流
//! 以 rtl_tcp 协议（RTL0 头 + 8 bit IQ）暴露给 SDR++/GNU Radio：
//!   rtl_tcp_bridge <COM 口> [监听端口，默认 1234]
//!
//! C3 教训记录：esp-radio 1.0.0-beta.1 的 C5 SoftAP 信标不发射
//! （官方未改动示例复现，见 docs/c5-registers.md §10），WiFi 直连
//! 暂缓，等上游修复后恢复 rtltcp 固件路线。

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};

use esp_sdr_shared::frame::{self, CaptureFrameHeader, FrameError, HEADER_LEN};

fn main() {
    let mut args = std::env::args().skip(1);
    let port_name = args
        .next()
        .unwrap_or_else(|| usage_and_exit("缺少 COM 口参数"));
    let listen_port: u16 = args
        .next()
        .map(|a| a.parse().unwrap_or_else(|_| usage_and_exit("端口必须是数字")))
        .unwrap_or(1234);

    let mut serial = serialport::new(&port_name, 115_200)
        .timeout(std::time::Duration::from_millis(200))
        .open()
        .unwrap_or_else(|error| panic!("打开 {port_name} 失败: {error}"));

    let listener = TcpListener::bind(("127.0.0.1", listen_port))
        .unwrap_or_else(|error| panic!("监听 {listen_port} 失败: {error}"));
    println!("桥已启动：{port_name} → 127.0.0.1:{listen_port}（rtl_tcp 协议，SDR++ 直接连接）");

    for stream in listener.incoming() {
        let mut client = match stream {
            Ok(stream) => stream,
            Err(error) => {
                println!("accept 失败: {error}");
                continue;
            }
        };
        client
            .set_nodelay(true)
            .unwrap_or_else(|error| println!("set_nodelay 失败: {error}"));
        println!("客户端接入 {}", client.peer_addr().map(|a| a.to_string()).unwrap_or_default());
        let result = serve_client(&mut serial, &mut client);
        println!("会话结束：{result:?}");
    }
}

fn serve_client(serial: &mut Box<dyn serialport::SerialPort>, client: &mut TcpStream) -> std::io::Result<()> {
    // 协议头："RTL0" + 调谐器类型 1(E4000) + 增益档数 0
    client.write_all(b"RTL0\x00\x00\x00\x01\x00\x00\x00\x00")?;
    client.flush()?;

    // 消费客户端命令（5 字节组；0x02 SET_FREQ——桥版本固定频点，仅丢弃）
    let mut command_junk = [0u8; 128];
    let _ = client.read(&mut command_junk);

    // 串口侧帧同步状态
    let mut ring: Vec<u8> = Vec::with_capacity(256 * 1024);
    let mut byte_buffer = [0u8; 16384];
    let mut frames_sent: u64 = 0;
    let mut crc_errors: u64 = 0;
    let started = std::time::Instant::now();
    let mut last_report = std::time::Instant::now();

    loop {
        // 客户端断开检测（非阻塞读，期待 0 字节与超时区分）
        client.set_nonblocking(true)?;
        match client.read(&mut command_junk) {
            Ok(0) => return Ok(()), // 客户端关闭
            Ok(_) => {}             // 忽略后续命令
            Err(error)
                if error.kind() == std::io::ErrorKind::WouldBlock
                    || error.kind() == std::io::ErrorKind::TimedOut => {}
            Err(error) => return Err(error),
        }
        client.set_nonblocking(false)?;

        // 串口读（200 ms 超时）
        let serial_read = serial.read(&mut byte_buffer);
        match serial_read {
            Ok(n) => ring.extend_from_slice(&byte_buffer[..n]),
            Err(error) if error.kind() == std::io::ErrorKind::TimedOut => {}
            Err(error) => return Err(std::io::Error::other(error.to_string())),
        }

        // 解出全部完整帧并转发
        loop {
            let Some(magic_offset) = find_magic(&ring) else {
                ring.clear();
                break;
            };
            ring.drain(..magic_offset);
            if ring.len() < HEADER_LEN {
                break;
            }
            let mut header_bytes = [0u8; HEADER_LEN];
            header_bytes.copy_from_slice(&ring[..HEADER_LEN]);
            let header = match CaptureFrameHeader::decode(&header_bytes) {
                Ok(header) => header,
                Err(FrameError::BadMagic) => continue,
                Err(error) => {
                    println!("帧头解码失败: {error:?}");
                    ring.remove(0);
                    continue;
                }
            };
            let payload_len = header.sample_word_count as usize * 4;
            if ring.len() < HEADER_LEN + payload_len {
                break; // 载荷未收齐
            }
            let payload = ring[HEADER_LEN..HEADER_LEN + payload_len].to_vec();
            let valid = header.verify_crc(&header_bytes, &payload);
            ring.drain(..HEADER_LEN + payload_len);
            if !valid {
                crc_errors += 1;
                continue; // CRC 错帧直接丢弃（SDR 容错）
            }

            // IQ8 打包（上游 pack_iq8 位序）并转发
            let mut packed = Vec::with_capacity(payload.len() / 2);
            for word in payload.chunks_exact(4) {
                let w = u32::from_le_bytes([word[0], word[1], word[2], word[3]]);
                packed.push(((w >> 2) & 0xFF) as u8);
                packed.push(((w >> 12) & 0xFF) as u8);
            }
            client.write_all(&packed)?;
            frames_sent += 1;
        }

        client.flush()?;
        if last_report.elapsed() >= std::time::Duration::from_secs(5) {
            println!(
                "已转发 {} 帧（CRC 错 {}），吞吐 {:.0} kS/s，运行 {:.0} s",
                frames_sent,
                crc_errors,
                frames_sent as f64 * 8190.0 / started.elapsed().as_secs_f64() / 1000.0,
                started.elapsed().as_secs_f64()
            );
            last_report = std::time::Instant::now();
        }
    }
}

fn find_magic(buffer: &[u8]) -> Option<usize> {
    buffer.windows(4).position(|window| window == frame::MAGIC)
}

fn usage_and_exit(message: &str) -> ! {
    eprintln!("{message}\n用法: rtl_tcp_bridge <COM 口> [监听端口，默认 1234]");
    std::process::exit(2);
}
