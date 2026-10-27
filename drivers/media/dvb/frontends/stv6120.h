/*
 * STV6120 dual satellite silicon tuner
 *
 * Register handling and tuning algorithm from the STMicroelectronics
 * stm_fe library (stv6120_tuner.c, GPLv2).
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */
#ifndef __STV6120_H
#define __STV6120_H

#include "stv6110x.h"	/* enum tuner_mode, enum tuner_status */

struct stv6120_config {
	u8	addr;		/* 7-bit I2C address of the chip */
	u32	refclk;		/* reference (crystal) clock [Hz] */
	u8	clk_div;	/* reference clock output divider: 1 or 2 */
	u8	tuner;		/* tuner of the chip: 1 or 2 */
	u8	rf_input;	/* RF input: 0..3 = A..D */
};

struct stv6120_devctl {
	int (*tuner_init)(struct dvb_frontend *fe);
	int (*tuner_sleep)(struct dvb_frontend *fe);
	int (*tuner_set_mode)(struct dvb_frontend *fe, enum tuner_mode mode);
	int (*tuner_set_frequency)(struct dvb_frontend *fe, u32 frequency);
	int (*tuner_get_frequency)(struct dvb_frontend *fe, u32 *frequency);
	int (*tuner_set_bandwidth)(struct dvb_frontend *fe, u32 bandwidth);
	int (*tuner_get_bandwidth)(struct dvb_frontend *fe, u32 *bandwidth);
	int (*tuner_set_bbgain)(struct dvb_frontend *fe, u32 gain);
	int (*tuner_get_bbgain)(struct dvb_frontend *fe, u32 *gain);
	int (*tuner_get_status)(struct dvb_frontend *fe, u32 *status);
};

#if defined(CONFIG_DVB_STV6120) || (defined(CONFIG_DVB_STV6120_MODULE) && defined(MODULE))

extern struct stv6120_devctl *stv6120_attach(struct dvb_frontend *fe,
					     const struct stv6120_config *config,
					     struct i2c_adapter *i2c);

#else
static inline struct stv6120_devctl *stv6120_attach(struct dvb_frontend *fe,
						    const struct stv6120_config *config,
						    struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}

#endif /* CONFIG_DVB_STV6120 */

#endif /* __STV6120_H */
