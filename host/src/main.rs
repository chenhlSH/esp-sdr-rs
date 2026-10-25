//! C2 主机端收谱工具：串口收帧 → CRC 校验 → FFT 平均频谱 → CSV。
//!
//! 用法：esp-sdr-host <COM 口> [收帧数量，默认 8]
//! 输出：spectrum.csv（频偏 Hz, 功率 dB）+ 控制台统计。

use esp_sdr_shared::frame::{self, CaptureFrameHeader, FrameError, HEADER_LEN};

const SAMPLE_RATE_HZ: f64 = 80_000_000.0;
const FFT_SIZE: usize = 4096;

fn main() {
    let mut args = std::env::args().skip(1);
    let source = args
        .next()
        .unwrap_or_else(|| usage_and_exit("缺少参数：COM 口 或 --file <路径>"));
    let frames_to_collect: usize = args
        .next()
        .map(|a| a.parse().unwrap_or_else(|_| usage_and_exit("帧数量必须是数字")))
        .unwrap_or(8);
    if frames_to_collect == 0 {
        usage_and_exit("帧数量必须大于 0");
    }

    // --file 模式：从抓包文件解码帧（诊断用；绕过串口层）
    if let Some(path) = source.strip_prefix("--file:") {
        run_file_mode(path, frames_to_collect);
        return;
    }
    let port_name = source;

    let mut port = serialport::new(&port_name, 115_200)
        .timeout(std::time::Duration::from_millis(1_000))
        .open()
        .unwrap_or_else(|error| panic!("打开 {port_name} 失败: {error}"));
    // USB-SJ 的 DTR/RTS 语义（espflash connection/reset.rs）：
    // DTR = IO0（BOOT 引脚！DTR=true = 下载模式），RTS = 复位。
    // 数据流与 DTR/RTS 电平无关（结束态两者皆 false 即可）。
    // 此处执行 espflash reset_after_flash 的 USB-SJ 分支：复位进正常运行。
    std::thread::sleep(std::time::Duration::from_millis(100));
    port.write_data_terminal_ready(false)
        .unwrap_or_else(|error| panic!("置 DTR 失败: {error}"));
    std::thread::sleep(std::time::Duration::from_millis(100));
    port.write_request_to_send(true)
        .unwrap_or_else(|error| panic!("置 RTS 失败: {error}"));
    port.write_data_terminal_ready(false)
        .unwrap_or_else(|error| panic!("置 DTR 失败: {error}"));
    port.write_request_to_send(true)
        .unwrap_or_else(|error| panic!("置 RTS 失败: {error}"));
    std::thread::sleep(std::time::Duration::from_millis(100));
    port.write_request_to_send(false)
        .unwrap_or_else(|error| panic!("清 RTS 失败: {error}"));

    // 无握手写：CDC 控制传输在 serialport 下不可靠（os error 121）。
    // 固件在启动 20 s 后自动开始采集（capburst 超时兜底），主机开端口即读，
    // 帧流在固件超时后自然到达。
    println!(
        "已打开 {port_name}，等待固件采集启动（约 20 s）与帧流（需要 {} 帧）...",
        frames_to_collect
    );

    let mut accumulated_power = vec![0f64; FFT_SIZE];
    let mut frames_ok: usize = 0;
    let mut crc_errors: usize = 0;
    let mut sequence_gaps: usize = 0;
    let mut last_sequence: Option<u16> = None;
    let mut header_hz: u32 = 0;

    let mut byte_buffer = [0u8; 4096];
    let mut ring: Vec<u8> = Vec::with_capacity(128 * 1024);
    let mut raw_bytes_total: u64 = 0;
    let mut last_report = std::time::Instant::now();
    let mut first_bytes: Vec<u8> = Vec::new();

    while frames_ok < frames_to_collect {
        let bytes_read = match port.read(&mut byte_buffer) {
            Ok(bytes) => bytes,
            // 读超时 = 窗口内无数据，继续轮询
            Err(error) if error.kind() == std::io::ErrorKind::TimedOut => {
                if last_report.elapsed() >= std::time::Duration::from_secs(5) {
                    println!(
                        "原始字节: {}，帧: {}/{}",
                        raw_bytes_total, frames_ok, frames_to_collect
                    );
                    last_report = std::time::Instant::now();
                }
                continue;
            }
            Err(error) => panic!("串口读失败: {error}"),
        };
        if bytes_read == 0 {
            continue;
        }
        if first_bytes.len() < 16 {
            first_bytes.extend_from_slice(&byte_buffer[..bytes_read.min(16 - first_bytes.len())]);
        }
        raw_bytes_total += bytes_read as u64;
        ring.extend_from_slice(&byte_buffer[..bytes_read]);

        // 同步并解出尽量多的完整帧
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
                Err(FrameError::BadMagic) => continue, // find_magic 保证不会发生
                Err(error) => {
                    println!("帧头解码失败: {error:?}，丢弃 1 字节重新同步");
                    ring.remove(0);
                    continue;
                }
            };
            let payload_len = header.sample_word_count as usize * 4;
            if ring.len() < HEADER_LEN + payload_len {
                break; // 载荷未收齐
            }

            let payload = ring[HEADER_LEN..HEADER_LEN + payload_len].to_vec();
            if !header.verify_crc(&header_bytes, &payload) {
                crc_errors += 1;
                ring.drain(..HEADER_LEN + payload_len);
                continue;
            }

            // 序列号连续性
            if let Some(expected) = last_sequence.map(|s| s.wrapping_add(1)) {
                if header.sequence != expected {
                    sequence_gaps += 1;
                    println!("序列跳变: 期望 {} 实际 {}", expected, header.sequence);
                }
            }
            last_sequence = Some(header.sequence);

            let words: Vec<u32> = payload
                .chunks_exact(4)
                .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                .collect();
            accumulate_spectrum(&words, &mut accumulated_power);

            frames_ok += 1;
            header_hz = header.frequency_mhz;
            println!(
                "帧 {}: {} 采样字, {} us, 频点 {} MHz（{}/{}）",
                header.sequence, header.sample_word_count, header.elapsed_us,
                header.frequency_mhz, frames_ok, frames_to_collect
            );
            ring.drain(..HEADER_LEN + payload_len);
        }
    }

    // 平均并输出 CSV
    let center_hz = f64::from(header_hz) * 1_000_000.0;
    let mut csv = String::with_capacity(FFT_SIZE * 48);
    csv.push_str("frequency_hz,power_db\n");
    let mut peaks: Vec<(usize, f64)> = Vec::new();
    for (bin_index, power) in accumulated_power.iter().enumerate() {
        let average = power / frames_ok as f64;
        let power_db = 10.0 * average.log10();
        // 复数 IQ：bin 0..N 经 fftshift 映射到 -fs/2..+fs/2
        let shifted = (bin_index + FFT_SIZE / 2) % FFT_SIZE;
        let offset_hz =
            (shifted as f64 - FFT_SIZE as f64 / 2.0) * SAMPLE_RATE_HZ / FFT_SIZE as f64;
        peaks.push((bin_index, power_db));
        csv.push_str(&format!("{:.0},{:.2}\n", center_hz + offset_hz, power_db));
    }
    std::fs::write("spectrum.csv", &csv).unwrap_or_else(|error| panic!("写 CSV 失败: {error}"));

    peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    println!("\n=== 统计 ===");
    println!(
        "原始字节: {}, 首字节: {}",
        raw_bytes_total,
        first_bytes.iter().map(|b| format!("{b:02x}")).collect::<String>()
    );
    println!("完整帧: {}, CRC 错误: {}, 序列跳变: {}", frames_ok, crc_errors, sequence_gaps);
    println!("频谱已写入 spectrum.csv（{} 个 bin，中心 {} MHz）", FFT_SIZE, header_hz);
    println!("前 5 峰（频偏 kHz, dB）:");
    for (bin_index, power_db) in peaks.iter().take(5) {
        let shifted = (bin_index + FFT_SIZE / 2) % FFT_SIZE;
        let offset_khz =
            (shifted as f64 - FFT_SIZE as f64 / 2.0) * SAMPLE_RATE_HZ / FFT_SIZE as f64 / 1_000.0;
        println!("  {:>10.1} kHz  {:>8.2} dB", offset_khz, power_db);
    }
}

fn usage_and_exit(message: &str) -> ! {
    eprintln!("{message}\n用法: esp-sdr-host <COM 口> [收帧数量，默认 8]  或  esp-sdr-host --file:<路径> [帧数]");
    std::process::exit(2);
}

/// --file 模式：从抓包文件解码帧并出频谱（诊断用，绕过串口层）。
fn run_file_mode(path: &str, frames_wanted: usize) {
    let data = std::fs::read(path).unwrap_or_else(|error| panic!("读文件失败: {error}"));
    println!("文件 {} 共 {} 字节", path, data.len());

    let mut accumulated_power = vec![0f64; FFT_SIZE];
    let mut frames_ok: usize = 0;
    let mut crc_errors: usize = 0;
    let mut header_hz: u32 = 0;
    let mut ring: Vec<u8> = Vec::new();
    ring.extend_from_slice(&data);

    while frames_ok < frames_wanted {
        let Some(magic_offset) = find_magic(&ring) else {
            println!("流末尾：无更多帧（累计 {}/{}）", frames_ok, frames_wanted);
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
            Err(error) => {
                println!("帧头解码失败: {error:?}");
                ring.remove(0);
                continue;
            }
        };
        let payload_len = header.sample_word_count as usize * 4;
        if ring.len() < HEADER_LEN + payload_len {
            println!("最后一帧载荷不完整（{}/{}），跳过", ring.len() - HEADER_LEN, payload_len);
            break;
        }
        let payload = ring[HEADER_LEN..HEADER_LEN + payload_len].to_vec();
        if !header.verify_crc(&header_bytes, &payload) {
            crc_errors += 1;
            println!("帧 {} CRC 错误", header.sequence);
            ring.drain(..HEADER_LEN + payload_len);
            continue;
        }
        let words: Vec<u32> = payload
            .chunks_exact(4)
            .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
            .collect();
        accumulate_spectrum(&words, &mut accumulated_power);
        frames_ok += 1;
        header_hz = header.frequency_mhz;
        println!(
            "帧 {}: {} 采样字, {} us, 频点 {} MHz（{}/{}）",
            header.sequence, header.sample_word_count, header.elapsed_us,
            header.frequency_mhz, frames_ok, frames_wanted
        );
        ring.drain(..HEADER_LEN + payload_len);
    }

    if frames_ok > 0 {
        write_spectrum_csv(&accumulated_power, frames_ok, header_hz);
    }
    println!(
        "完整帧: {}, CRC 错误: {}",
        frames_ok, crc_errors
    );
}

/// 输出平均频谱 CSV 与峰值统计。
fn write_spectrum_csv(accumulated_power: &[f64], frames_ok: usize, header_hz: u32) {
    let center_hz = f64::from(header_hz) * 1_000_000.0;
    let mut csv = String::with_capacity(accumulated_power.len() * 48);
    csv.push_str("frequency_hz,power_db\n");
    let mut peaks: Vec<(usize, f64)> = Vec::new();
    for (bin_index, power) in accumulated_power.iter().enumerate() {
        let average = power / frames_ok as f64;
        let power_db = 10.0 * average.log10();
        // 复数 IQ：bin 0..N 经 fftshift 映射到 -fs/2..+fs/2
        let shifted = (bin_index + accumulated_power.len() / 2) % accumulated_power.len();
        let offset_hz =
            (shifted as f64 - accumulated_power.len() as f64 / 2.0) * SAMPLE_RATE_HZ
                / accumulated_power.len() as f64;
        peaks.push((bin_index, power_db));
        csv.push_str(&format!("{:.0},{:.2}\n", center_hz + offset_hz, power_db));
    }
    std::fs::write("spectrum.csv", &csv).unwrap_or_else(|error| panic!("写 CSV 失败: {error}"));

    peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    println!(
        "频谱已写入 spectrum.csv（{} 个 bin，中心 {} MHz）",
        accumulated_power.len(),
        header_hz
    );
    println!("前 5 峰（频偏 kHz, dB）:");
    for (bin_index, power_db) in peaks.iter().take(5) {
        let shifted = (bin_index + accumulated_power.len() / 2) % accumulated_power.len();
        let offset_khz = (shifted as f64 - accumulated_power.len() as f64 / 2.0)
            * SAMPLE_RATE_HZ
            / accumulated_power.len() as f64
            / 1_000.0;
        println!("  {:>10.1} kHz  {:>8.2} dB", offset_khz, power_db);
    }
}

fn find_magic(buffer: &[u8]) -> Option<usize> {
    buffer.windows(4).position(|window| window == frame::MAGIC)
}

/// 把一帧采样字累加进平均频谱：切 4096 点段做 radix-2 FFT，累加功率谱。
fn accumulate_spectrum(words: &[u32], accumulated_power: &mut [f64]) {
    let chunk_count = words.len() / FFT_SIZE;
    for chunk_index in 0..chunk_count {
        let chunk = &words[chunk_index * FFT_SIZE..(chunk_index + 1) * FFT_SIZE];
        let mut real = vec![0f32; FFT_SIZE];
        let mut imag = vec![0f32; FFT_SIZE];
        for (index, &word) in chunk.iter().enumerate() {
            let (i_sample, q_sample) = esp_sdr_shared::iq::unpack_word(word);
            real[index] = i_sample as f32;
            imag[index] = q_sample as f32;
        }
        fft_inplace(&mut real, &mut imag);
        for (bin_index, (re, im)) in real.iter().zip(&imag).enumerate() {
            accumulated_power[bin_index] += f64::from(re * re + im * im);
        }
    }
}

/// 迭代式 radix-2 FFT（DIT，位反转 + 蝶形），n 必须是 2 的幂。
fn fft_inplace(real: &mut [f32], imag: &mut [f32]) {
    let n = real.len();
    assert!(n.is_power_of_two(), "FFT 长度必须是 2 的幂");

    // 位反转重排
    let mut j = 0usize;
    for i in 0..n - 1 {
        if i < j {
            real.swap(i, j);
            imag.swap(i, j);
        }
        let mut m = n >> 1;
        while m >= 1 && j & m != 0 {
            j ^= m;
            m >>= 1;
        }
        j |= m;
    }

    // 蝶形
    let mut length = 2usize;
    while length <= n {
        let step = length / 2;
        let angle_step = -2.0 * core::f64::consts::PI / length as f64;
        for start in (0..n).step_by(length) {
            for k in 0..step {
                let angle = angle_step * k as f64;
                let twiddle_re = angle.cos() as f32;
                let twiddle_im = angle.sin() as f32;
                let i = start + k;
                let m = i + step;
                let temp_re = real[m] * twiddle_re - imag[m] * twiddle_im;
                let temp_im = real[m] * twiddle_im + imag[m] * twiddle_re;
                real[m] = real[i] - temp_re;
                imag[m] = imag[i] - temp_im;
                real[i] += temp_re;
                imag[i] += temp_im;
            }
        }
        length <<= 1;
    }
}
