/*
 * IDL4K: TS inputs of the four demodulator paths on the STx7108 TANGO TP
 * and STFE (drivers/media/dvb/tango).
 *
 * Kept in a header so that an out-of-tree test module can register the
 * same device on a kernel without it.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2.
 */
#ifndef __IDL4K_TANGO_PTI_H
#define __IDL4K_TANGO_PTI_H

#include <linux/platform_device.h>
#include <linux/stm/pad.h>
#include <linux/stm/pio-control.h>
#include <linux/stm/stx7108.h>
#include <linux/stm/platform.h>
#include <media/tango_pti.h>

/*
 * TSIN pads: PIO10[0-7], PIO11[0-7], PIO12[0-3], alternate function 1,
 * inputs.  Four of them are TS clocks (retimed as clocks, inverted), the
 * rest data and control retimed on clock 0.
 */
#define IDL4K_TSIN_DATA(_port, _pin) \
	{ \
		.gpio = stm_gpio(_port, _pin), \
		.direction = stm_pad_gpio_direction_in, \
		.function = 1, \
		.priv = &(struct stx7108_pio_config) { \
			.retime = &(struct stm_pio_control_retime_config) { \
				.retime = 1, \
				.clk1notclk0 = 0, \
				.clknotdata = 0, \
				.double_edge = 0, \
				.invertclk = 0, \
				.delay_input = 0, \
			}, \
		}, \
	}

#define IDL4K_TSIN_CLK(_port, _pin) \
	{ \
		.gpio = stm_gpio(_port, _pin), \
		.direction = stm_pad_gpio_direction_in, \
		.function = 1, \
		.priv = &(struct stx7108_pio_config) { \
			.retime = &(struct stm_pio_control_retime_config) { \
				.retime = 0, \
				.clk1notclk0 = 0, \
				.clknotdata = 1, \
				.double_edge = 0, \
				.invertclk = 1, \
				.delay_input = 0, \
			}, \
		}, \
	}

static struct stm_pad_config idl4k_tsin_pad_config = {
	.gpios_num = 20,
	.gpios = (struct stm_pad_gpio []) {
		IDL4K_TSIN_DATA(10, 0),
		IDL4K_TSIN_DATA(10, 1),
		IDL4K_TSIN_DATA(10, 2),
		IDL4K_TSIN_CLK(10, 3),
		IDL4K_TSIN_DATA(10, 4),
		IDL4K_TSIN_DATA(10, 5),
		IDL4K_TSIN_DATA(10, 6),
		IDL4K_TSIN_DATA(10, 7),
		IDL4K_TSIN_CLK(11, 0),
		IDL4K_TSIN_DATA(11, 1),
		IDL4K_TSIN_DATA(11, 2),
		IDL4K_TSIN_DATA(11, 3),
		IDL4K_TSIN_DATA(11, 4),
		IDL4K_TSIN_CLK(11, 5),
		IDL4K_TSIN_DATA(11, 6),
		IDL4K_TSIN_DATA(11, 7),
		IDL4K_TSIN_DATA(12, 0),
		IDL4K_TSIN_DATA(12, 1),
		IDL4K_TSIN_CLK(12, 2),
		IDL4K_TSIN_DATA(12, 3),
	},
};

static struct platform_device idl4k_tango_pti_device = {
	.name = "tango-pti",
	.id = -1,
	.num_resources = 4,
	.resource = (struct resource []) {
		STM_PLAT_RESOURCE_MEM_NAMED("tp", 0xfd600000, 0x2f000),
		STM_PLAT_RESOURCE_MEM_NAMED("stfe", 0xfda40000, 0x1800),
		STM_PLAT_RESOURCE_MEM_NAMED("stfe-ram", 0xfea00000, 0x4000),
		STM_PLAT_RESOURCE_IRQ_NAMED("tp", 142, -1),
	},
	.dev.platform_data = &(struct tango_pti_platform_data) {
		.num_inputs = 4,
		/* frontend n -> IB; fe0 -> IB1 is verified, the others follow
		 * the vendor demux numbering (1, 0, 3, 2) */
		.input_ib = { 1, 0, 3, 2 },
		/* serial, byte endianness, async, align on SOP, inverted clock */
		.input_format = 0x1f,
		.pad_config = &idl4k_tsin_pad_config,
		.clk_name = "CLKA_TP",
		.firmware = "pti_tp2.elf",
	},
};

#endif
