//! ESP32-C5 射频初始化与控制层。
//!
//! 职责：PHY 启动（esp-phy 最小直启）、vendor blob 函数绑定
//! （phy_set_chanfreq / force_rx_gain 等，见 vendor/README.md）、
//! 增益锁定维护任务、发射强制接口。
//! 全部公开 API 以 Result 返回，显式错误枚举定义故障模型。
//!
//! 当前状态：C0 骨架，实现自 C1 检查点起逐项落地。
