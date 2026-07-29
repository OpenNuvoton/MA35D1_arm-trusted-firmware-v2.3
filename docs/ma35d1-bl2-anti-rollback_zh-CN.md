# BL2 Anti-Rollback（OTP 熔丝版本计数器）

## 1. 概述

这是 BL2 的固件防回滚（anti-rollback）保护机制。BL2 启动时，会将编译时固化的版本号与芯片 OTP（One-Time Programmable）熔丝中记录的版本计数器进行比较：

- 如果当前 BL2 比芯片中已烧录的版本更旧，则判定为回滚攻击（或误刷旧版本），输出错误信息并停止启动（panic()）。
- 如果当前 BL2 比 OTP 中记录的版本更新，则将 OTP 计数器提升到该新版本并记录下来。
- 如果版本相同，则不执行任何操作。

**支持平台**：目前仅支持 **MA35D1** 与 **MA35D05K** 系列。MA35D05K 使用与 MA35D1 相同的平台代码，仅搭配不同的 Device Tree。**暂不支持 MA35D0 与 MA35H0 系列**。

## 2. 使用说明

### 2.1 启用/关闭

该功能由构建参数 `OTP_ANTI_ROLLBACK` 控制，默认关闭（0）。

启用示例：

```bash
make CROSS_COMPILE=aarch64-linux- PLAT=ma35d1 OTP_ANTI_ROLLBACK=1 bl2
```

也可直接修改 `platform.mk` 将默认值改为 1。

> 警告：该操作会永久烧录 OTP 熔丝，不可逆。

### 2.2 发布新版本时

修改：

```c
#define MA35D1_BL2_FW_VERSION_STRING "ma35-tfa-v1.2.0"
#define MA35D1_BL2_FW_VERSION_COUNT 6U
```

- VERSION_STRING：仅用于日志显示。
- VERSION_COUNT：修复需要防回滚的安全问题时必须递增。

### 2.3 启动日志

```text
BL2 firmware version: ma35-tfa-v1.2.0 (count=6)
OTP fuse counter: 6
```

检测到回滚时系统会报错并 panic()；发现新版本时会更新 OTP 计数器；OTP 读取或烧录失败时采用 fail-secure 策略并直接停止启动。

### 2.4 使用限制

- OTP 总容量为 704 次版本递增。
- 该机制必须建立在 Secure Boot 已验证 BL2 镜像真实性的前提下。

## 3. 技术原理

### 3.1 相关文件

- `ma35d1_otp_fuse.c/h`：OTP 计数器 API。
- `tsi_cmd.c` / `tsi_cmd.h`：TSI 与 OTP 通讯接口。
- `ma35d1_bl2_el3_setup.c`：版本检查逻辑。
- `platform.mk`：功能开关与编译设置。

### 3.2 OTP Secure Region 配置

OTP Secure Region 中 0x120~0x174（22 个 32-bit word）保留给版本计数器。

### 3.3 Bit-Walk（温度计编码）

OTP 只能由 0 编程到 1，因此采用温度计编码：每增加一个版本就烧录一个新的 bit。

- 每个 word 从 bit0 到 bit31。
- 从最高地址 0x174 开始向低地址写入。
- 当前计数值等于所有已烧录 bit 数量总和。

### 3.4 容量上限

22 × 32 = 704。

### 3.5 读取与烧录流程

读取时统计全部 word 的 popcount 总和；烧录时逐个 bit 编程直到达到目标计数值。底层通过 TSI 安全处理器执行 OTP 命令。

### 3.6 Fail-Secure 设计

任何 OTP 读取、通信或烧录失败都会直接 panic()，避免未验证状态继续启动。

### 3.7 与 TSI Patch Image 的关系

BL2 初始化时会加载 Nuvoton 提供的 `tsi_patch_image[]`，以支持扩展 OTP 指令。

### 3.8 既有行为说明

- `ma35d1_config_setup()` 在 BL2 与 BL31 阶段各执行一次。
- OP-TEE 可能拥有独立的 TSI 初始化流程，与 BL2 防回滚机制互不影响。
