// 主机端 rtl_tcp 客户端：连接板卡 AP 网关的 1234 端口，读协议头 + IQ 流，
// 统计吞吐与连续时长（C3 验收：60 秒无断流）。
// 用法：esp-sdr-host --rtltcp <IP> [秒数，默认 60]
#![allow(dead_code)]

use std::io::{Read, Write};
use std::net::TcpStream;

fn main() {
    let mut args = std::env::args().skip(1);
    let mode = args.next().unwrap_or_default();
    if mode != "--rtltcp" {
        eprintln!("用法: esp-sdr-host --rtltcp <板卡IP> [秒数，默认 60]");
        std::process::exit(2);
    }
    let ip = args.next().unwrap_or_else(|| {
        eprintln!("缺少板卡 IP");
        std::process::exit(2);
    });
    let seconds: u64 = args.next().map(|s| s.parse().unwrap_or(60)).unwrap_or(60);
    let addr = format!("{ip}:1234");

    let mut stream = TcpStream::connect(&addr)
        .unwrap_or_else(|error| panic!("连接 {addr} 失败: {error}"));
    stream
        .set_read_timeout(Some(std::time::Duration::from_secs(5)))
        .unwrap();
    println!("已连接 {addr}");

    // 协议头 12 字节："RTL0" + tuner(u32 BE) + gain_count(u32 BE)
    let mut header = [0u8; 12];
    read_exact_or_panic(&mut stream, &mut header);
    assert_eq!(&header[0..4], b"RTL0", "协议头魔数不匹配");
    let tuner = u32::from_be_bytes([header[4], header[5], header[6], header[7]]);
    println!("协议头 OK：tuner=0x{tuner:x}，开始收流 {seconds} 秒");

    // 发送 SET_FREQ 2412 MHz（客户端命令 0x02 + u32 BE Hz）——驱动板卡真实调谐
    let freq_hz: u32 = 2_412_000_000;
    let command: [u8; 5] = [
        0x02,
        (freq_hz >> 24) as u8,
        (freq_hz >> 16) as u8,
        (freq_hz >> 8) as u8,
        freq_hz as u8,
    ];
    stream
        .write_all(&command)
        .unwrap_or_else(|error| panic!("发 SET_FREQ 失败: {error}"));

    // 收流统计
    let mut buffer = vec![0u8; 65536];
    let mut total_bytes: u64 = 0;
    let mut read_calls: u64 = 0;
    let mut max_gap_ms: u64 = 0;
    let started = std::time::Instant::now();
    let mut last_data = started;
    let mut first_data_at: Option<std::time::Instant> = None;

    while started.elapsed() < std::time::Duration::from_secs(seconds) {
        match stream.read(&mut buffer) {
            Ok(0) => {
                println!("连接关闭（EOF）于 {} s", started.elapsed().as_secs());
                break;
            }
            Ok(n) => {
                if first_data_at.is_none() {
                    first_data_at = Some(std::time::Instant::now());
                    println!("首包数据到达于 {:.1} ms", started.elapsed().as_secs_f64() * 1000.0);
                }
                total_bytes += n as u64;
                read_calls += 1;
                let gap = last_data.elapsed().as_millis() as u64;
                if gap > max_gap_ms {
                    max_gap_ms = gap;
                }
                last_data = std::time::Instant::now();
            }
            Err(error) if error.kind() == std::io::ErrorKind::WouldBlock
                || error.kind() == std::io::ErrorKind::TimedOut =>
            {
                // 5 秒读超时 = 断流
                println!("读超时（断流）于 {} s", started.elapsed().as_secs());
                break;
            }
            Err(error) => panic!("读失败: {error}"),
        }
    }
    let elapsed = started.elapsed().as_secs_f64();
    let rate_ksps = total_bytes as f64 / 2.0 / elapsed / 1000.0;
    println!("\n=== 验收统计 ===");
    println!("持续: {elapsed:.1} s");
    println!("总字节: {total_bytes}（≈ {rate_ksps:.0} kS/s IQ8）");
    println!("读调用: {read_calls}，最大间隔: {max_gap_ms} ms");
    if max_gap_ms <= 1_000 && total_bytes > 1_000_000 {
        println!("结论：60 秒流连续，无断流 —— 通过");
    } else {
        println!("结论：存在断流或数据不足 —— 未通过");
        std::process::exit(1);
    }
}

fn read_exact_or_panic(stream: &mut TcpStream, buffer: &mut [u8]) {
    let mut filled = 0;
    while filled < buffer.len() {
        match stream.read(&mut buffer[filled..]) {
            Ok(0) => panic!("连接在协议头期间关闭"),
            Ok(n) => filled += n,
            Err(error) => panic!("读协议头失败: {error}"),
        }
    }
}
