/*
 * STV6120 dual satellite silicon tuner
 *
 * Register handling and tuning algorithm from the STMicroelectronics
 * stm_fe library (stv6120_tuner.c, GPLv2).
 *
 * The chip has two tuners sharing one I2C address, a reference clock
 * output and four RF inputs (A..D) that either tuner can select.  Each
 * tuner is attached to its own frontend; the two share one register
 * shadow.  The demodulator serialises the I2C access of both tuners
 * (it opens its I2C repeater around every call).
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/i2c.h>

#include "dvb_frontend.h"
#include "stv6120.h"

static int verbose;
module_param(verbose, int, 0644);
MODULE_PARM_DESC(verbose, "Set verbosity level");

#define dprintk(fmt, arg...) do {					\
	if (verbose)							\
		printk(KERN_DEBUG "stv6120: " fmt, ##arg);		\
} while (0)

/* Common registers */
#define STV6120_CTRL1		0x00	/* K[7:3] RDIV[2] ODIV[1] MCLKDIV[0] */
#define STV6120_CTRL9		0x09	/* RFSEL_2[3:2] RFSEL_1[1:0] */
#define STV6120_CTRL10		0x0a	/* LNA D..A on [5:2], PATHON_2[1], PATHON_1[0] */
#define STV6120_NREGS		0x19

/* Per tuner registers: tuner 1 at the offset, tuner 2 at offset + 10 */
#define STV6120_CTRL2		0x01	/* DCLOOPOFF[7] SDOFF[6] SYN[5] REFOUTSEL[4] BBGAIN[3:0] */
#define STV6120_CTRL3		0x02	/* NDIV[7:0] */
#define STV6120_CTRL4		0x03	/* F[6:0] in [7:1], NDIV[8] in [0] */
#define STV6120_CTRL5		0x04	/* F[14:7] */
#define STV6120_CTRL6		0x05	/* ICP[6:4] VCOILOW[3] F[17:15] in [2:0] */
#define STV6120_CTRL7		0x06	/* RCCLFOFF[7] PDIV[6:5] CF[4:0] */
#define STV6120_CTRL8		0x07	/* CALTIME[5] CFHF[4:0] */
#define STV6120_STAT		0x08	/* CALVCOSTRT[2] CALRCSTRT[1] LOCK[0] */

#define STV6120_SDOFF		0x40
#define STV6120_SYN		0x20
#define STV6120_CALVCOSTRT	0x04
#define STV6120_CALRCSTRT	0x02
#define STV6120_LOCK		0x01

/* Power-on values (stm_fe), all LNAs off */
static const u8 stv6120_defaults[STV6120_NREGS] = {
	0x75, 0x33, 0xce, 0x54, 0x55, 0x0d, 0x32, 0x44, 0x0e,
	0xf9, 0x03, 0x33, 0xce, 0x54, 0x55, 0x0d, 0x32, 0x44,
	0x0e, 0x00, 0x00, 0x4c, 0x00, 0x00, 0x4c,
};

/* VCO range [kHz] -> charge pump current */
static const struct {
	u32 low, high;
	u8 icp;
} stv6120_icp[] = {
	{ 2300000, 2388000, 0 },
	{ 2389000, 2610000, 1 },
	{ 2611000, 2919000, 2 },
	{ 2920000, 3273000, 3 },
	{ 3274000, 3715000, 5 },
	{ 3716000, 4246000, 6 },
	{ 4247000, 4600000, 7 },
};

struct stv6120_chip {
	struct list_head	list;
	struct i2c_adapter	*i2c;
	u8			addr;
	int			users;
	int			initialised;
	u32			refclk;
	u8			regs[STV6120_NREGS];
	u8			input[2];	/* RF input of each tuner */
	u8			active[2];	/* tuner woken up */
};

struct stv6120_state {
	struct stv6120_chip		*chip;
	const struct stv6120_config	*config;
	int				t;	/* tuner index 0 / 1 */
	struct stv6120_devctl		devctl;
};

static LIST_HEAD(stv6120_chips);
static DEFINE_MUTEX(stv6120_chips_lock);

static inline u8 treg(int t, u8 reg)
{
	return reg + t * 10;
}

static int stv6120_write(struct stv6120_chip *chip, u8 start, int len)
{
	u8 buf[STV6120_NREGS + 1];
	struct i2c_msg msg = {
		.addr = chip->addr, .flags = 0, .buf = buf, .len = len + 1
	};
	int ret;

	buf[0] = start;
	memcpy(&buf[1], &chip->regs[start], len);
	ret = i2c_transfer(chip->i2c, &msg, 1);
	if (ret != 1) {
		printk(KERN_ERR "stv6120: I2C write 0x%02x/%d at 0x%02x failed (%d)\n",
		       start, len, chip->addr, ret);
		return -EREMOTEIO;
	}
	return 0;
}

static int stv6120_read(struct stv6120_chip *chip, u8 start, int len)
{
	u8 b0 = start;
	struct i2c_msg msg[] = {
		{ .addr = chip->addr, .flags = 0, .buf = &b0, .len = 1 },
		{ .addr = chip->addr, .flags = I2C_M_RD,
		  .buf = &chip->regs[start], .len = len },
	};
	int ret;

	ret = i2c_transfer(chip->i2c, msg, 2);
	if (ret != 2) {
		printk(KERN_ERR "stv6120: I2C read 0x%02x/%d at 0x%02x failed (%d)\n",
		       start, len, chip->addr, ret);
		return -EREMOTEIO;
	}
	return 0;
}

/* Start a calibration (STAT bit) and wait up to 10 ms for its end */
static int stv6120_calibrate(struct stv6120_chip *chip, int t, u8 bit)
{
	u8 stat = treg(t, STV6120_STAT);
	int i, ret;

	chip->regs[stat] |= bit;
	ret = stv6120_write(chip, stat, 1);
	for (i = 0; !ret && i < 10; i++) {
		ret = stv6120_read(chip, stat, 1);
		if (ret || !(chip->regs[stat] & bit))
			break;
		msleep(1);
	}
	chip->regs[stat] &= ~bit;
	return ret;
}

/* RF input of each tuner; LNA of every input used by a woken-up tuner,
 * path of every woken-up tuner on */
static int stv6120_update_inputs(struct stv6120_chip *chip)
{
	u8 lna = 0;
	int t;

	for (t = 0; t < 2; t++)
		if (chip->active[t])
			lna |= 4 << chip->input[t];
	chip->regs[STV6120_CTRL9] = (chip->regs[STV6120_CTRL9] & 0xf0) |
				    chip->input[1] << 2 | chip->input[0];
	chip->regs[STV6120_CTRL10] = (chip->regs[STV6120_CTRL10] & 0xc0) | lna |
				     (chip->active[1] ? 2 : 0) |
				     (chip->active[0] ? 1 : 0);
	return stv6120_write(chip, STV6120_CTRL9, 2);
}

static int stv6120_set_mode(struct dvb_frontend *fe, enum tuner_mode mode)
{
	struct stv6120_state *state = fe->tuner_priv;
	struct stv6120_chip *chip = state->chip;
	u8 ctrl2 = treg(state->t, STV6120_CTRL2);
	int ret;

	dprintk("0x%02x tuner %d %s\n", chip->addr, state->t + 1,
		mode == TUNER_SLEEP ? "sleep" : "wake");
	if (mode == TUNER_SLEEP) {
		chip->regs[ctrl2] = (chip->regs[ctrl2] & ~STV6120_SYN) | STV6120_SDOFF;
		chip->active[state->t] = 0;
	} else {
		chip->regs[ctrl2] = (chip->regs[ctrl2] & ~STV6120_SDOFF) | STV6120_SYN;
		chip->active[state->t] = 1;
	}
	ret = stv6120_write(chip, ctrl2, 1);
	if (!ret)
		ret = stv6120_update_inputs(chip);
	return ret;
}

static int stv6120_sleep(struct dvb_frontend *fe)
{
	return stv6120_set_mode(fe, TUNER_SLEEP);
}

static int stv6120_init(struct dvb_frontend *fe)
{
	struct stv6120_state *state = fe->tuner_priv;
	struct stv6120_chip *chip = state->chip;
	const struct stv6120_config *config = state->config;
	int ret;

	chip->input[state->t] = config->rf_input & 3;
	if (!chip->initialised) {
		u8 *ctrl1 = &chip->regs[STV6120_CTRL1];

		memcpy(chip->regs, stv6120_defaults, STV6120_NREGS);
		/* K = reference clock in MHz - 16, R = 2 from 27 MHz up */
		*ctrl1 = (*ctrl1 & 0x07) | ((chip->refclk / 1000000 - 16) << 3);
		if (chip->refclk >= 27000000)
			*ctrl1 |= 0x04;
		else
			*ctrl1 &= ~0x04;
		if (config->clk_div == 2)
			*ctrl1 |= 0x02;
		else
			*ctrl1 &= ~0x02;
		ret = stv6120_write(chip, 0, STV6120_NREGS);
		if (ret)
			return ret;
		chip->initialised = 1;
		printk(KERN_INFO "stv6120: chip at 0x%02x initialised, %u MHz reference\n",
		       chip->addr, chip->refclk / 1000000);
	}
	return stv6120_set_mode(fe, TUNER_WAKE);
}

static u32 stv6120_tuner_freq(struct stv6120_chip *chip, int t)
{
	const u8 *r = &chip->regs[t * 10];
	u32 ndiv, frac, ref, freq;

	ndiv = (r[STV6120_CTRL4] & 0x01) << 8 | r[STV6120_CTRL3];
	frac = (r[STV6120_CTRL6] & 0x07) << 15 | r[STV6120_CTRL5] << 7 |
	       r[STV6120_CTRL4] >> 1;
	ref = chip->refclk / 1000 >> ((chip->regs[STV6120_CTRL1] >> 2) & 1);
	freq = ref * ndiv + (ref * frac) / 0x40000;
	return freq >> (1 + ((r[STV6120_CTRL7] >> 5) & 3));
}

static int stv6120_get_frequency(struct dvb_frontend *fe, u32 *frequency)
{
	struct stv6120_state *state = fe->tuner_priv;
	struct stv6120_chip *chip = state->chip;
	int ret;

	ret = stv6120_read(chip, treg(state->t, STV6120_CTRL3), 5);
	if (ret)
		return ret;
	*frequency = stv6120_tuner_freq(chip, state->t);
	return 0;
}

/* frequency in kHz */
static int stv6120_set_frequency(struct dvb_frontend *fe, u32 frequency)
{
	struct stv6120_state *state = fe->tuner_priv;
	struct stv6120_chip *chip = state->chip;
	int t = state->t, o = !t;
	u32 rdiv = (chip->regs[STV6120_CTRL1] >> 2) & 1;
	u32 ref = chip->refclk / 1000;
	u32 p, p2, f2, vco, vco2, delta, n, frac;
	u8 *r = &chip->regs[t * 10];
	unsigned int i;

	if (frequency <= 325000)
		p = 3;
	else if (frequency <= 650000)
		p = 2;
	else if (frequency < 1151000)
		p = 1;
	else
		p = 0;

	/* keep the two VCOs apart: move by 2 MHz if they come too close */
	f2 = stv6120_tuner_freq(chip, o);
	p2 = (chip->regs[o * 10 + STV6120_CTRL7] >> 5) & 3;
	vco = frequency << (p + 1);
	vco2 = f2 << (p2 + 1);
	delta = 4000 << abs((int)p2 - (int)p);
	if (chip->active[o] && abs((int)vco - (int)vco2) < delta) {
		u32 base = p > p2 ? f2 / (delta / 4000) : f2 * (delta / 4000);

		frequency = vco > vco2 ? base + 2000 : base - 2000;
		dprintk("tuner %d moved to %u kHz, VCO too close to tuner %d\n",
			t + 1, frequency, o + 1);
	}

	/* N = F * P * R / Fref, fraction in 1 / 2^18 */
	frac = frequency << (p + rdiv + 1);
	n = frac / ref;
	frac %= ref;
	if (frac < 0x3fff)
		frac = (0x40000 * frac) / ref;
	else if (frac < 0x3fff0)
		frac = (0x8000 * frac) / (ref / 8);
	else
		frac = (0x2000 * frac) / (ref / 32);

	for (i = 0; i < ARRAY_SIZE(stv6120_icp) - 1; i++)
		if (vco >= stv6120_icp[i].low && vco <= stv6120_icp[i].high)
			break;

	r[STV6120_CTRL3] = n & 0xff;
	r[STV6120_CTRL4] = (frac & 0x7f) << 1 | ((n >> 8) & 1);
	r[STV6120_CTRL5] = (frac >> 7) & 0xff;
	r[STV6120_CTRL6] = (r[STV6120_CTRL6] & 0x08) | stv6120_icp[i].icp << 4 |
			   ((frac >> 15) & 7);
	r[STV6120_CTRL7] = (r[STV6120_CTRL7] & 0x9f) | p << 5;

	dprintk("0x%02x tuner %d: %u kHz, P %u, N %u, F %u, ICP %u\n",
		chip->addr, t + 1, frequency, p, n, frac, stv6120_icp[i].icp);

	if (stv6120_write(chip, treg(t, STV6120_CTRL3), 5))
		return -EREMOTEIO;
	return stv6120_calibrate(chip, t, STV6120_CALVCOSTRT);
}

/* low pass filter, bandwidth in Hz (both sides) */
static int stv6120_set_bandwidth(struct dvb_frontend *fe, u32 bandwidth)
{
	struct stv6120_state *state = fe->tuner_priv;
	struct stv6120_chip *chip = state->chip;
	u8 ctrl7 = treg(state->t, STV6120_CTRL7);
	u32 cf;

	if (bandwidth / 2 > 36000000)
		cf = 31;
	else if (bandwidth / 2 < 5000000)
		cf = 0;
	else
		cf = bandwidth / 2 / 1000000 - 5;

	chip->regs[ctrl7] = (chip->regs[ctrl7] & ~0x1f) | cf;
	if (stv6120_write(chip, ctrl7, 1))
		return -EREMOTEIO;
	return stv6120_calibrate(chip, state->t, STV6120_CALRCSTRT);
}

static int stv6120_get_bandwidth(struct dvb_frontend *fe, u32 *bandwidth)
{
	struct stv6120_state *state = fe->tuner_priv;
	struct stv6120_chip *chip = state->chip;
	u8 ctrl7 = treg(state->t, STV6120_CTRL7);
	int ret;

	ret = stv6120_read(chip, ctrl7, 1);
	if (ret)
		return ret;
	*bandwidth = ((chip->regs[ctrl7] & 0x1f) + 5) * 2000000;
	return 0;
}

/* base band gain in dB, 0..30 in 2 dB steps */
static int stv6120_set_bbgain(struct dvb_frontend *fe, u32 gain)
{
	struct stv6120_state *state = fe->tuner_priv;
	struct stv6120_chip *chip = state->chip;
	u8 ctrl2 = treg(state->t, STV6120_CTRL2);

	chip->regs[ctrl2] = (chip->regs[ctrl2] & ~0x0f) | ((gain / 2) & 0x0f);
	return stv6120_write(chip, ctrl2, 1);
}

static int stv6120_get_bbgain(struct dvb_frontend *fe, u32 *gain)
{
	struct stv6120_state *state = fe->tuner_priv;
	struct stv6120_chip *chip = state->chip;
	u8 ctrl2 = treg(state->t, STV6120_CTRL2);
	int ret;

	ret = stv6120_read(chip, ctrl2, 1);
	if (ret)
		return ret;
	*gain = 2 * (chip->regs[ctrl2] & 0x0f);
	return 0;
}

static int stv6120_get_status(struct dvb_frontend *fe, u32 *status)
{
	struct stv6120_state *state = fe->tuner_priv;
	struct stv6120_chip *chip = state->chip;
	u8 stat = treg(state->t, STV6120_STAT);
	int ret;

	ret = stv6120_read(chip, stat, 1);
	if (ret)
		return ret;
	*status = (chip->regs[stat] & STV6120_LOCK) ? TUNER_PHASELOCKED : 0;
	return 0;
}

static int stv6120_release(struct dvb_frontend *fe)
{
	struct stv6120_state *state = fe->tuner_priv;

	if (!state)
		return 0;
	mutex_lock(&stv6120_chips_lock);
	if (--state->chip->users == 0) {
		list_del(&state->chip->list);
		kfree(state->chip);
	}
	mutex_unlock(&stv6120_chips_lock);
	fe->tuner_priv = NULL;
	kfree(state);
	return 0;
}

static struct dvb_tuner_ops stv6120_ops = {
	.info = {
		.name		= "STV6120",
		.frequency_min	=  950000,
		.frequency_max	= 2150000,
	},
	.release		= stv6120_release,
};

struct stv6120_devctl *stv6120_attach(struct dvb_frontend *fe,
				      const struct stv6120_config *config,
				      struct i2c_adapter *i2c)
{
	struct stv6120_state *state;
	struct stv6120_chip *chip;

	if (config->tuner < 1 || config->tuner > 2)
		return NULL;
	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return NULL;

	mutex_lock(&stv6120_chips_lock);
	list_for_each_entry(chip, &stv6120_chips, list)
		if (chip->i2c == i2c && chip->addr == config->addr)
			goto found;
	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		mutex_unlock(&stv6120_chips_lock);
		kfree(state);
		return NULL;
	}
	chip->i2c = i2c;
	chip->addr = config->addr;
	chip->refclk = config->refclk;
	memcpy(chip->regs, stv6120_defaults, STV6120_NREGS);
	list_add(&chip->list, &stv6120_chips);
found:
	chip->users++;
	mutex_unlock(&stv6120_chips_lock);

	state->chip = chip;
	state->config = config;
	state->t = config->tuner - 1;
	state->devctl.tuner_init = stv6120_init;
	state->devctl.tuner_sleep = stv6120_sleep;
	state->devctl.tuner_set_mode = stv6120_set_mode;
	state->devctl.tuner_set_frequency = stv6120_set_frequency;
	state->devctl.tuner_get_frequency = stv6120_get_frequency;
	state->devctl.tuner_set_bandwidth = stv6120_set_bandwidth;
	state->devctl.tuner_get_bandwidth = stv6120_get_bandwidth;
	state->devctl.tuner_set_bbgain = stv6120_set_bbgain;
	state->devctl.tuner_get_bbgain = stv6120_get_bbgain;
	state->devctl.tuner_get_status = stv6120_get_status;

	fe->tuner_priv = state;
	fe->ops.tuner_ops = stv6120_ops;

	printk(KERN_INFO "stv6120: tuner %d of 0x%02x attached, RF input %c\n",
	       config->tuner, config->addr, 'A' + (config->rf_input & 3));
	return &state->devctl;
}
EXPORT_SYMBOL(stv6120_attach);

MODULE_DESCRIPTION("STV6120 dual satellite tuner");
MODULE_LICENSE("GPL");
