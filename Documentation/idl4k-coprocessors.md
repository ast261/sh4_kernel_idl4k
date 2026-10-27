# IDL4K: co-processors for CSA descrambling

Status of the investigation into moving DVB-CSA descrambling off the host
CPU.  Last updated 2026-10-27.  Nothing here is implemented yet.


## 1. Problem

oscam works on the box.  The control words go to minisatip, which
descrambles with libdvbcsa on the host CPU.  That limits the box to **two
encrypted channels at a time**.

libdvbcsa is already built the best way for SH-4: buildroot
(`/home/axel/software/idl4k/image/sh4-buildroot-glibc-2.23`) configures it
with `DVBCSA_USE_UINT32` and `-O3 -funroll-loops`.  The only untried option
is `--enable-alt-sbox` (a few percent at most).  More channels need another
core, or hardware descrambling.


## 2. Processors on the STx7108

From the product brief (`specs/STI71_709639.pdf`) and `/proc/clocks` on
box 2:

| CPU | Clock | Current use |
|---|---|---|
| ST40-300 host | 600 MHz (`CLKA_SH4_ICK`) | Linux |
| ST40-300 real-time ("RT") | not visible in `/proc/clocks` | unused, presumably held in reset |
| ST231 video (`CLKA_LX_DMU_CPU`) | 500 MHz | unusable (user) |
| ST231 audio (`CLKA_LX_AUD_CPU`) | 500 MHz | unusable (user) |
| ST231 security (`CLKA_LX_SEC_CPU`) | 500 MHz | unused; the brief calls it the "additional ST231 … for multimedia, security and networking tasks" |

The kernel already has `stm-coprocessor` with three ST231 entries
(`/dev/st231-0..2`).  `/proc/coprocessor` shows all three "not allocated!"
because the boot arguments carry no `coprocessor_mem=`.


## 3. Routes

| Route | Capacity | Toolchain | Status |
|---|---|---|---|
| TP secure co-processor (hardware CSA) | all channels | none | **closed** (3.1) |
| RT ST40 | about +2 channels | sh4 gcc 4.9.4, libdvbcsa unchanged | **preferred**; boot mechanism unknown (3.2) |
| Security ST231 | unknown | proprietary ST200 toolset, not available here | unlikely (3.3) |

### 3.1 TP secure co-processor

`internet/oscam/liboscam_stapi5.a` (taken from NCam) descrambles in
hardware: it loads control words with `STTKD_KeyAllocate/KeyWrite/CipherSet/
PathAllocate` and routes the PIDs with `STPTI_DeviceSetPidSecurityPath`.
No ST231 is involved.

On the IDL4K this route is closed:

- The stock firmware has no STTKD module (no `/dev/stapi/sttkd_ioctl`).
- Stock `stapi_core` boots the TP with `TP_FIRMWARE_BYPASS_SECURE_COPRO`.
- The GPL `crypto_engine-modules-15.2.1` (r7oss) does not drive the key
  hardware.  Its only back end ("Tango/STTKD") sends Multicom MME commands
  to closed firmware.
- The key hardware (TKD) is undocumented.

### 3.2 RT ST40 (preferred)

Same SH-4 instruction set as the host, MMU, 32K/32K caches.  A bare-metal
program built with the existing toolchain could run libdvbcsa unchanged.
The host would pass TS batches and control words through a shared-memory
ring in bigphysarea; minisatip would hand batches over instead of calling
`dvbcsa_bs_decrypt` itself.

What exists:

- The kernel can run on either core: `STX7108_HOST_CORE` in
  `include/linux/stm/stx7108.h` (CCN_PRR bit 7), RT interrupt controllers
  ILC3 at 0xfda3c000 and INTC2 at 0xfda38000
  (`arch/sh/kernel/cpu/sh4/setup-stx7108.c`).
- The U-Boot romgen script (`u-boot/board/st/idl4k/idl4k-stx7108.romgen`)
  was generated with `dual_cpu=1, active_cores=host:rt`.  That is ST's
  JTAG tooling, which is not available.

What is missing: how to release the RT core from reset and set its boot
address.  Not in any kernel branch (`stm24_0210..0217`, `duckbox-master`),
not in the IDL4K U-Boot (which never touches the RT core), not in the r7oss
trees.

### 3.3 Security ST231

- No ST200 compiler on this machine (only `specs/st200-toolset.pdf`); the
  ST200 toolset, OS21 and the ST231 side of Multicom are proprietary.  A GPL
  setup would need a bare-metal program and a home-made shared-memory ring.
- As the security CPU it is the most likely of the three to enforce signed
  firmware or to be firewalled from memory.  If that is why the audio and
  video ST231s cannot be used, it probably applies here too.
- libdvbcsa's bitslice code uses plain integer registers; a 4-issue VLIW
  without SIMD may gain something per clock over the SH-4, but this is
  unmeasured.

Boot plumbing exists in `drivers/stm/copro-st_socs.c`: boot address in
bank 0 CFG6/7/8, reset (active low) in bank 0 CFG0 bits 4/5/6.


## 4. Sysconf scan (box 2, 2026-10-27, read-only)

Bank 0 at 0xfde30000 is the CPU control bank.  Register layout (from
U-Boot `include/asm-sh/stx7108reg.h`): STA0 at 0x00, CFGn at 0x04 + 4n for
n = 0..8, CFG9–11 do not exist, CFG12 at 0x28, CFG13 at 0x2c, CFG14 at 0x30.

| Register | Address | Value | Meaning |
|---|---|---|---|
| STA0 | 0xfde30000 | 0x00010CEB | unknown |
| CFG0 | 0xfde30004 | 0x0000000D | bits 4–6: ST231 resets, all 0 = all three held in reset.  Bits 0, 2, 3 set; **bit 1 clear: candidate for the RT reset** |
| CFG1 | 0xfde30008 | 0x00000002 | unknown |
| CFG2 | 0xfde3000c | 0x0000000F | unknown |
| CFG3 | 0xfde30010 | 0x00000001 | unknown |
| CFG4 | 0xfde30014 | 0x00000000 | not used by any driver; **candidate ST40 boot address** |
| CFG5 | 0xfde30018 | 0x00000000 | not used by any driver; **candidate ST40 boot address** |
| CFG6–8 | 0xfde3001c–24 | 0x00000000 | ST231 boot addresses |
| CFG12 | 0xfde30028 | 0xFFFFFFFF | bit 30 SATA1 HC reset, bit 31 MiPHY0 reset (released); romgen pulses bits 1 and 2 at boot |
| CFG13 | 0xfde3002c | 0xFFFFFFFF | bit 0 MiPHY1 reset (released) |
| CFG14 | 0xfde30030 | 0x00000BB8 | unknown |

Banks 1–4 (0xfde20000, 0xfda50000, 0xfd500000, 0xfe700000) hold device ID,
pin, clock and peripheral settings; nothing CPU related was found.

Read with `devmem ADDR 32` on the box.


## 5. Next step

Write experiment on box 2 (needs the user's OK; may hang the box):

1. Put a small SH-4 loop in bigphysarea that increments a counter in
   uncached memory.
2. Set CFG5 (second try: CFG4) to the loop's physical address.
3. Set CFG0 bit 1.
4. Watch the counter from the host.

Risk: if the guess is wrong, the RT core may start from the fixed reset
vector (boot flash), run U-Boot, reinitialise clocks and the RAM
controller, and hang the host.  Recovery is a power cycle; box 2 has a
UART for the log.

If the RT core can be started this way, the remaining work is ordinary:
memory map and caches on the RT side, the host/RT ring, a mailbox or
polling for signalling, and the minisatip hand-over.
