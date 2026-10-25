// 链接 vendor 的 librftest.a（force_rx_gain / adctrig / dactrig 所在库）。
// 仅对 riscv32 固件目标生效；来源与校验见 vendor/README.md。
use std::env;
use std::path::Path;

fn main() {
    let target = env::var("TARGET").unwrap_or_default();
    if !target.starts_with("riscv32") {
        return;
    }
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap_or_default();
    let vendor_dir = Path::new(&manifest_dir).join("../../vendor");
    println!("cargo:rustc-link-search=native={}", vendor_dir.display());
    println!("cargo:rustc-link-lib=static=rftest");
    println!("cargo:rerun-if-changed={}", vendor_dir.join("librftest.a").display());
}
