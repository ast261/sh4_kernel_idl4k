/*
 * TANGO transport processor (TP) and STFE live inputs of the STx7108,
 * driven with the Transport Engine TP firmware.
 *
 * The board registers a "tango-pti" platform device with
 *   IORESOURCE_MEM "tp"        TP (STxP70) registers and memories
 *   IORESOURCE_MEM "stfe"      STFE registers
 *   IORESOURCE_MEM "stfe-ram"  STFE internal RAM
 *   IORESOURCE_IRQ "tp"        TP mailbox interrupt
 * and struct tango_pti_platform_data.
 *
 * A client (the DVB demux glue) gets an input, adds PIDs to it and receives
 * the matching 188-byte packets from a kernel thread.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2.
 */
#ifndef __MEDIA_TANGO_PTI_H
#define __MEDIA_TANGO_PTI_H

#include <linux/types.h>

#define TANGO_PTI_MAX_INPUTS	6
#define TANGO_PTI_ALL_PIDS	0x2000	/* the whole transport stream */

struct stm_pad_config;

struct tango_pti_platform_data {
	int num_inputs;
	/* STFE input block (IB) fed by each input */
	u8 input_ib[TANGO_PTI_MAX_INPUTS];
	/* IB INPUT_FORMAT_CONFIG value (serial/parallel, clock edge, ...) */
	u32 input_format;
	/* TSIN pads; claimed at probe */
	struct stm_pad_config *pad_config;
	const char *clk_name;		/* TP clock */
	const char *firmware;		/* TP firmware image */
};

struct tango_pti_input;

/* Called from the driver's thread with whole packets, in order */
typedef void (*tango_pti_data_fn)(void *priv, const u8 *packets, unsigned int count);

struct tango_pti_input *tango_pti_input_get(unsigned int n, tango_pti_data_fn fn, void *priv);
void tango_pti_input_put(struct tango_pti_input *in);

/* PIDs are reference counted; the input runs while at least one is set */
int tango_pti_pid_add(struct tango_pti_input *in, u16 pid);
void tango_pti_pid_remove(struct tango_pti_input *in, u16 pid);

#endif
