// 链接 vendor 的 librftest.a（adctrig 所在库）与 libbttestmode.a
// （librftest shell 的 BT 测试符号依赖，见 vendor/README.md）。
// 仅对 riscv32 固件目标生效。
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
    // 顺序重要：librftest 引用 libbttestmode 的符号，后者必须排在其后
    println!("cargo:rustc-link-lib=static=rftest");
    println!("cargo:rustc-link-lib=static=bttestmode");
    println!("cargo:rerun-if-changed={}", vendor_dir.join("librftest.a").display());
    println!("cargo:rerun-if-changed={}", vendor_dir.join("libbttestmode.a").display());
}
