# BL2 Anti-Rollback (OTP Fuse Version Counter)

## 1. Overview

This document describes the BL2 firmware anti-rollback protection mechanism. During boot, BL2 compares its built-in firmware version with the version counter stored in the chip's OTP (One-Time Programmable) fuse region.

- If the running BL2 image is older than the version already recorded in OTP, it is treated as a rollback attempt and the system halts with `panic()`.
- If the running BL2 image is newer, the OTP counter is increased to the new version.
- If both versions match, no action is taken.

**Supported platforms:** MA35D1 and MA35D05K. MA35D05K reuses the MA35D1 platform implementation with a different device tree. MA35D0 and MA35H0 are currently not supported.

## 2. Usage

### 2.1 Enable or Disable

The feature is controlled by the build option `OTP_ANTI_ROLLBACK`, which is disabled by default.

Example:

```bash
make CROSS_COMPILE=aarch64-linux- PLAT=ma35d1 OTP_ANTI_ROLLBACK=1 bl2
```

> Warning: Enabling this feature permanently programs OTP fuse bits and cannot be reversed.

### 2.2 Releasing a New Version

Update:

```c
#define MA35D1_BL2_FW_VERSION_STRING "ma35-tfa-v1.2.0"
#define MA35D1_BL2_FW_VERSION_COUNT 6U
```

- `VERSION_STRING` is for log display only.
- `VERSION_COUNT` must be incremented whenever a security-relevant fix requires rollback protection.

### 2.3 Boot Messages

```text
BL2 firmware version: ma35-tfa-v1.2.0 (count=6)
OTP fuse counter: 6
```

Rollback detection, OTP read failure, and OTP programming failure all cause boot termination via `panic()`.

### 2.4 Limitations

- The OTP version counter can only be increased 704 times during device lifetime.
- This mechanism only provides meaningful protection when Secure Boot verifies BL2 authenticity.

## 3. Technical Details

### 3.1 Relevant Files

- `ma35d1_otp_fuse.c/h` : OTP counter APIs.
- `tsi_cmd.c` and `tsi_cmd.h` : Low-level TSI commands.
- `ma35d1_bl2_el3_setup.c` : Version comparison and rollback checks.
- `platform.mk` : Build switch integration.

### 3.2 OTP Secure Region Layout

Addresses 0x120 through 0x174 (22 words, 32 bits each) are reserved for the version counter.

### 3.3 Bit-Walk (Thermometer Code)

Because OTP bits can only transition from 0 to 1, the counter uses thermometer encoding.

- Bits are programmed from LSB to MSB within a word.
- Programming starts at address 0x174 and proceeds toward 0x120.
- The counter value equals the total number of programmed bits.

### 3.4 Maximum Capacity

22 words × 32 bits = 704 version increments.

### 3.5 Read and Program Flow

The read operation sums the popcount of all 22 words. Programming is performed one bit at a time until the requested counter value is reached. All OTP access is handled through the TSI secure co-processor.

### 3.6 Fail-Secure Design

Any OTP communication, read, or programming error causes an immediate `panic()` and prevents boot continuation.

### 3.7 Relationship with the TSI Patch Image

BL2 loads the Nuvoton-provided `tsi_patch_image[]` to enable extended OTP commands such as `TSI_OTP_Read()` and `TSI_OTP_Program()`.

### 3.8 Existing Behaviors

- `ma35d1_config_setup()` is executed independently by BL2 and BL31.
- OP-TEE may perform its own TSI initialization and patch loading sequence, which is independent of the BL2 anti-rollback mechanism.
