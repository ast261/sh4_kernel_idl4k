# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Repository Is

Linux kernel 2.6.32.42 for the **Inverto IDL4K IPTV set-top box**, targeting the STMicroelectronics STx7108 SoC on SH-4 (SuperH 32-bit RISC) architecture. This is a heavily customized fork from STMicroelectronics' internal tree (`_stm24_0208`), with Inverto-specific board support layered on top.

Local version string: `-idl4k_7108`

## Build Commands

Cross-compilation is required (this is SH-4, not the host arch). Set `ARCH=sh` and `CROSS_COMPILE` to the appropriate toolchain prefix.

```bash
# Load the IDL4K preset configuration
make ARCH=sh idl4k_defconfig

# Build the kernel
make ARCH=sh CROSS_COMPILE=sh4-linux- -j$(nproc)

# Interactive config editing
make ARCH=sh menuconfig

# Clean (preserves .config)
make ARCH=sh clean

# Full clean including config
make ARCH=sh mrproper

# Build with verbose output (useful for debugging build issues)
make ARCH=sh V=1
```

The IDL4K defconfig sets `CONFIG_INITRAMFS_SOURCE="rootfs-idl4k.cpio"` — a rootfs CPIO must exist at that path for a complete build.

## Key Architecture

### SH-4 / IDL4K Board Support

- **`arch/sh/boards/mach-idl4k/`** — IDL4K board initialization. Configures STx7108 peripherals: STMMAC Gigabit Ethernet in GMII mode, GPIO-based PHY speed control, NAND/SPI flash, I2C, RTC, LEDs.
- **`arch/sh/configs/idl4k_defconfig`** — the primary defconfig.
- **`arch/sh/boards/mach-stx7108/`** — generic STx7108 reference board (upstream of mach-idl4k).

### STMicroelectronics Custom Subsystem

`drivers/stm/` is a large non-mainline subsystem specific to STx series SoCs. Key drivers:

| File | Role |
|---|---|
| `stx7108.c` | SoC platform device registration (clocks, pinmux, peripherals) |
| `stx7108_comms.c` | UART/SSC/SPI communication subsystems |
| `gpio.c` | STM GPIO controller |
| `fdma.c` | FastDMA controller |
| `pcie.c` | PCIe support |
| `pms.c` | Power management (LPM/standby) |
| `stm-coprocessor.c` | Audio/video co-processor firmware loader |
| `mali/` | Mali GPU driver |

### STMMAC Ethernet Driver

`drivers/net/stmmac/` — the Gigabit MAC driver used for network access. IDL4K-specific tuning disables scatter-gather and sets RX/TX ring buffers to 512 entries. Realtek PHY patches are applied for reliability.

### Other Notable Drivers

- `drivers/char/lirc/lirc_stm.c` — infrared remote control receiver
- `drivers/ata/ahci_stm.c`, `sata_stm.c` — SATA
- `drivers/usb/host/hcd-stm.c` — USB host
- `drivers/media/` — DVB stack with multistream and DTMB extensions (recent commits)

### Closed-Source Kernel Modules

The IDL4K board uses a **STV6120** dual satellite tuner, **STV0900** demodulator, and **LNBH24** LNB power controller. These components are driven by closed-source kernel modules not present in this repository.

### I2C Bus Layout

Two I2C buses are exposed by the STx7108 SSC controllers:

**i2c-stm0 (bus 0):**
| Address | Device |
|---|---|
| 0x08–0x0b | 4× LNBH24 LNB power controllers (quad-tuner capable) |
| 0x68–0x69 | 2× STV0900 demodulators |

**i2c-stm1 (bus 1):**
| Address | Device |
|---|---|
| 0x50 | EEPROM (board config / MAC address), claimed by kernel driver |

The STV6120 tuners are not directly visible on either bus — they sit behind the STV0900's integrated I2C repeater and are accessed via the demodulator.

The following closed-source modules are required for full functionality. The `stapi_*` modules originate from STMicroelectronics; the `axe_*` modules originate from **Inverto Digital Labs**.

| Module | Role |
|---|---|
| `stapi_core` | STAPI core runtime bundle: OS HAL, STEVT event manager, STAVMEM AV memory partitioner, STBUFFER, STTBX debug infrastructure, PIO GPIO abstraction, STI2C Linux bridge, TS/PES injection, STAPI network device, STSYS |
| `stapi_ioctl` | Userspace ioctl interface to the STAPI framework; bundles ioctl front-ends for STAVMEM, STBUFFER, STI2C, STFDMA, TS/PES injection, STAPI network device, and STSYS |
| `axe_dmx` | Inverto Digital Labs demux driver |
| `axe_dmxts` | Inverto Digital Labs demux-ts driver |
| `axe_fp` | Inverto Digital Labs front panel / satellite-to-IP driver (fp-s2i) |
| `axe_fe` | Inverto Digital Labs frontend driver |
| `axe_i2c` | Inverto Digital Labs I2C driver (tuner/demodulator access) |

## Branch Strategy

- `master` — IDL4K production base
- `stm24_0210` … `stm24_0217` — upstream STM release branches (periodic rebase targets)
- `*-no-nand` variants — builds without NAND for testing
- `idl4k-update` — IDL4K platform updates
- `duckbox-master` — alternative DVB/receiver integration

## Sparse Static Analysis

```bash
make ARCH=sh C=1   # check only recompiled files
make ARCH=sh C=2   # check all files
```
