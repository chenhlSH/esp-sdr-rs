// 把块 1 让渡断言接入链接流程（esp_sdr_guard.ld），使违规构建在链接期失败。
use std::env;
use std::path::Path;

fn main() {
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    let ld_dir = Path::new(&manifest_dir).join("ld");
    println!("cargo:rustc-link-search={}", ld_dir.display());
    println!("cargo:rustc-link-arg=-Tesp_sdr_guard.ld");
    println!("cargo:rerun-if-changed={}", ld_dir.join("esp_sdr_guard.ld").display());
}
