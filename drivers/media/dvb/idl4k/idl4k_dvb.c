/*
 * Inverto IDL4K: one DVB adapter per satellite input.
 *
 * Two STV0900 dual demodulators (I2C 0x68, 0x69), each with an STV6120
 * dual tuner behind its I2C repeater (0x60, 0x63), and one LNBH24 per RF
 * input.  Frontend 2n is path 1 of demodulator n and uses tuner 2 of its
 * STV6120, frontend 2n + 1 is path 2 with tuner 1.  Frontend n takes RF
 * input n unless the rf_input parameter says otherwise.
 *
 * The LNB of RF input n is the LNBH24 of input n; its 22 kHz / DiSEqC
 * signal comes from the demodulator path of frontend n.  So the voltage,
 * tone and DiSEqC of a frontend go to the LNB supply and the demodulator
 * path of its RF input, not to its own.  The LNB supply of an input stays
 * on while any frontend on that input wants it.  While it is in use,
 * another frontend on that input gets -EBUSY for a different voltage or
 * tone, and its DiSEqC is dropped: either would switch the LNB polarisation
 * or band under the running stream.
 *
 * The demux of each adapter programs PID filters on the TANGO transport
 * processor (tango-pti) and gets the filtered packets back into the
 * software demux.
 *
 * Each frontend has an LED trigger, idl4k-fe0 .. idl4k-fe3, that is on
 * from the first tune of the frontend until its last close for writing;
 * the board makes it the default trigger of the front panel tuner LED.
 * Not from the open: minisatip opens all frontends at startup.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/stm/gpio.h>
#include <media/tango_pti.h>

#include "dvbdev.h"
#include "dvb_demux.h"
#include "dmxdev.h"
#include "dvb_frontend.h"
#include "stv6120.h"	/* before stv090x.h: enum tuner_mode */
#include "stv090x.h"
#include "lnbh24.h"

#define IDL4K_INPUTS	4
#define IDL4K_I2C_BUS	0

/* reset of demodulator 0 / 1, active low */
static const unsigned int idl4k_demod_reset[2] = {
	stm_gpio(9, 4), stm_gpio(9, 5)
};
static const u8 idl4k_demod_addr[2] = { 0x68, 0x69 };
static const u8 idl4k_tuner_addr[2] = { 0x60, 0x63 };
static const u8 idl4k_lnb_addr[IDL4K_INPUTS] = { 0x08, 0x0a, 0x09, 0x0b };

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static int rf_input[IDL4K_INPUTS] = { 0, 1, 2, 3 };
module_param_array(rf_input, int, NULL, 0444);
MODULE_PARM_DESC(rf_input, "RF input (0..3 = A..D) of each frontend");

struct idl4k_adapter {
	int index;
	struct dvb_adapter adapter;
	struct dvb_demux demux;
	struct dmxdev dmxdev;
	struct dmx_frontend hw_frontend;
	struct dmx_frontend mem_frontend;
	struct tango_pti_input *input;
	struct stv090x_config demod_config;
	struct stv6120_config tuner_config;
	struct dvb_frontend *fe;
	char led_name[12];
	struct led_trigger *led;
	enum dvbfe_search (*search)(struct dvb_frontend *fe,
				    struct dvb_frontend_parameters *p);

	/* LNB supply of this frontend's RF input, and the voltage and tone
	 * this frontend has set on it */
	int (*set_voltage)(struct dvb_frontend *fe, fe_sec_voltage_t voltage);
	fe_sec_voltage_t voltage;
	fe_sec_tone_mode_t tone;

	/* tone / DiSEqC of this demodulator path, for the LNB of input index */
	int (*set_tone)(struct dvb_frontend *fe, fe_sec_tone_mode_t tone);
	int (*send_master_cmd)(struct dvb_frontend *fe,
			       struct dvb_diseqc_master_cmd *cmd);
	int (*send_burst)(struct dvb_frontend *fe, fe_sec_mini_cmd_t burst);
};

static struct idl4k_adapter *adapters[IDL4K_INPUTS];
static struct i2c_adapter *idl4k_i2c;
static DEFINE_MUTEX(idl4k_lnb_lock);

static int idl4k_start_feed(struct dvb_demux_feed *feed)
{
	struct idl4k_adapter *a = feed->demux->priv;

	return tango_pti_pid_add(a->input, feed->pid);
}

static int idl4k_stop_feed(struct dvb_demux_feed *feed)
{
	struct idl4k_adapter *a = feed->demux->priv;

	tango_pti_pid_remove(a->input, feed->pid);
	return 0;
}

static void idl4k_data(void *priv, const u8 *packets, unsigned int count)
{
	struct idl4k_adapter *a = priv;

	dvb_dmx_swfilter_packets(&a->demux, packets, count);
}

/* adapter whose demodulator path drives the LNB of a's RF input */
static struct idl4k_adapter *idl4k_lnb_owner(struct dvb_frontend *fe)
{
	struct idl4k_adapter *a = fe->dvb->priv;

	return adapters[a->tuner_config.rf_input];
}

/* another frontend on a's RF input with the LNB supply on, if any;
 * called with idl4k_lnb_lock held */
static struct idl4k_adapter *idl4k_lnb_user(struct idl4k_adapter *a)
{
	int n;

	for (n = 0; n < IDL4K_INPUTS; n++)
		if (adapters[n] && adapters[n] != a &&
		    adapters[n]->tuner_config.rf_input ==
		    a->tuner_config.rf_input &&
		    adapters[n]->voltage != SEC_VOLTAGE_OFF)
			return adapters[n];
	return NULL;
}

/* Frontends on the same RF input share its LNB supply: only switch it off
 * when none of them wants it any more, and refuse another voltage while
 * one of them uses it */
static int idl4k_set_voltage(struct dvb_frontend *fe, fe_sec_voltage_t voltage)
{
	struct idl4k_adapter *a = fe->dvb->priv;
	struct idl4k_adapter *u;
	fe_sec_voltage_t v = voltage;
	int ret;

	mutex_lock(&idl4k_lnb_lock);
	u = idl4k_lnb_user(a);
	if (u && voltage == SEC_VOLTAGE_OFF)
		v = u->voltage;
	if (u && voltage != SEC_VOLTAGE_OFF && voltage != u->voltage) {
		ret = -EBUSY;
	} else {
		ret = a->set_voltage(fe, v);
		if (!ret)
			a->voltage = voltage;
	}
	mutex_unlock(&idl4k_lnb_lock);
	return ret;
}

static int idl4k_set_tone(struct dvb_frontend *fe, fe_sec_tone_mode_t tone)
{
	struct idl4k_adapter *a = fe->dvb->priv;
	struct idl4k_adapter *o = idl4k_lnb_owner(fe);
	struct idl4k_adapter *u;
	int ret;

	if (!o)
		return -ENODEV;
	mutex_lock(&idl4k_lnb_lock);
	u = idl4k_lnb_user(a);
	if (u && tone != u->tone) {
		ret = -EBUSY;
	} else {
		ret = o->set_tone(o->fe, tone);
		if (!ret)
			a->tone = tone;
	}
	mutex_unlock(&idl4k_lnb_lock);
	return ret;
}

/* Last close of the frontend: it no longer uses the LNB and its LED goes
 * off.  dvb-core only switches the supply off when the frontend thread
 * exits, which can be a minute later, and the other frontends on the input
 * would get -EBUSY until then.  The supply itself stays as it is until
 * that point. */
static int idl4k_ts_bus_ctrl(struct dvb_frontend *fe, int acquire)
{
	struct idl4k_adapter *a = fe->dvb->priv;

	if (!acquire) {
		led_trigger_event(a->led, LED_OFF);
		mutex_lock(&idl4k_lnb_lock);
		a->voltage = SEC_VOLTAGE_OFF;
		mutex_unlock(&idl4k_lnb_lock);
	}
	return 0;
}

/* tuning switches the LED on; dvb-core calls this again while it searches */
static enum dvbfe_search idl4k_search(struct dvb_frontend *fe,
				      struct dvb_frontend_parameters *p)
{
	struct idl4k_adapter *a = fe->dvb->priv;

	led_trigger_event(a->led, LED_FULL);
	return a->search(fe, p);
}

/* DiSEqC while another frontend uses the LNB is dropped */
static int idl4k_lnb_busy(struct idl4k_adapter *a)
{
	int busy;

	mutex_lock(&idl4k_lnb_lock);
	busy = idl4k_lnb_user(a) != NULL;
	mutex_unlock(&idl4k_lnb_lock);
	return busy;
}

static int idl4k_send_master_cmd(struct dvb_frontend *fe,
				 struct dvb_diseqc_master_cmd *cmd)
{
	struct idl4k_adapter *o = idl4k_lnb_owner(fe);

	if (idl4k_lnb_busy(fe->dvb->priv))
		return 0;
	return o ? o->send_master_cmd(o->fe, cmd) : -ENODEV;
}

static int idl4k_send_burst(struct dvb_frontend *fe, fe_sec_mini_cmd_t burst)
{
	struct idl4k_adapter *o = idl4k_lnb_owner(fe);

	if (idl4k_lnb_busy(fe->dvb->priv))
		return 0;
	return o ? o->send_burst(o->fe, burst) : -ENODEV;
}

static int idl4k_frontend_attach(struct idl4k_adapter *a)
{
	struct stv090x_config *dc = &a->demod_config;
	struct stv6120_config *tc = &a->tuner_config;
	struct stv6120_devctl *ctl;
	int chip = a->index / 2, path = a->index % 2;

	dc->device = STV0900;
	dc->demod_mode = STV090x_DUAL;
	dc->clk_mode = STV090x_CLK_EXT;
	dc->xtal = 30000000;
	dc->address = idl4k_demod_addr[chip];
	dc->ts1_mode = STV090x_TSMODE_SERIAL_CONTINUOUS;
	dc->ts2_mode = STV090x_TSMODE_SERIAL_CONTINUOUS;
	dc->ts_serial_ts1_ts2 = 1;
	dc->repeater_level = STV090x_RPTLEVEL_32;
	dc->tuner_on_rpt1 = 1;

	a->fe = stv090x_attach(dc, idl4k_i2c,
			       path ? STV090x_DEMODULATOR_1 : STV090x_DEMODULATOR_0);
	if (!a->fe)
		return -ENODEV;

	tc->addr = idl4k_tuner_addr[chip];
	tc->refclk = 30000000;
	tc->clk_div = 1;
	tc->tuner = path ? 1 : 2;
	tc->rf_input = rf_input[a->index] & 3;
	ctl = stv6120_attach(a->fe, tc, idl4k_i2c);
	if (!ctl)
		goto err;
	dc->tuner_init = ctl->tuner_init;
	dc->tuner_sleep = ctl->tuner_sleep;
	dc->tuner_set_mode = ctl->tuner_set_mode;
	dc->tuner_set_frequency = ctl->tuner_set_frequency;
	dc->tuner_get_frequency = ctl->tuner_get_frequency;
	dc->tuner_set_bandwidth = ctl->tuner_set_bandwidth;
	dc->tuner_get_bandwidth = ctl->tuner_get_bandwidth;
	dc->tuner_get_status = ctl->tuner_get_status;

	if (!lnbh24_attach(a->fe, idl4k_i2c, LNBH24_TTX, 0,
			   idl4k_lnb_addr[tc->rf_input]))
		goto err;

	a->set_voltage = a->fe->ops.set_voltage;
	a->set_tone = a->fe->ops.set_tone;
	a->send_master_cmd = a->fe->ops.diseqc_send_master_cmd;
	a->send_burst = a->fe->ops.diseqc_send_burst;
	a->search = a->fe->ops.search;
	a->fe->ops.set_voltage = idl4k_set_voltage;
	a->fe->ops.ts_bus_ctrl = idl4k_ts_bus_ctrl;
	a->fe->ops.set_tone = idl4k_set_tone;
	a->fe->ops.diseqc_send_master_cmd = idl4k_send_master_cmd;
	a->fe->ops.diseqc_send_burst = idl4k_send_burst;
	a->fe->ops.search = idl4k_search;

	/* demodulator path and tuner run from reset until the first open;
	 * put both into standby like the original axe_fe does at load */
	if (a->fe->ops.sleep(a->fe) < 0)
		printk(KERN_WARNING "idl4k_dvb: frontend %d standby failed\n",
		       a->index);

	if (dvb_register_frontend(&a->adapter, a->fe) < 0)
		goto err;
	return 0;

err:
	dvb_frontend_detach(a->fe);
	a->fe = NULL;
	return -ENODEV;
}

static void idl4k_adapter_remove(struct idl4k_adapter *a)
{
	struct dmx_demux *dmx = &a->demux.dmx;

	if (a->fe) {
		dvb_unregister_frontend(a->fe);
		dvb_frontend_detach(a->fe);
	}
	dmx->remove_frontend(dmx, &a->mem_frontend);
	dmx->remove_frontend(dmx, &a->hw_frontend);
	dvb_dmxdev_release(&a->dmxdev);
	dvb_dmx_release(&a->demux);
	tango_pti_input_put(a->input);
	dvb_unregister_adapter(&a->adapter);
	led_trigger_event(a->led, LED_OFF);
	led_trigger_unregister_simple(a->led);
	kfree(a);
}

static struct idl4k_adapter *idl4k_adapter_add(int n)
{
	struct idl4k_adapter *a;
	struct dmx_demux *dmx;
	int err;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return ERR_PTR(-ENOMEM);
	a->index = n;
	a->voltage = SEC_VOLTAGE_OFF;
	a->tone = SEC_TONE_OFF;
	snprintf(a->led_name, sizeof(a->led_name), "idl4k-fe%d", n);
	led_trigger_register_simple(a->led_name, &a->led);

	err = dvb_register_adapter(&a->adapter, "IDL4K", THIS_MODULE, NULL, adapter_nr);
	if (err < 0)
		goto err_led;
	a->adapter.priv = a;

	a->input = tango_pti_input_get(n, idl4k_data, a);
	if (IS_ERR(a->input)) {
		err = PTR_ERR(a->input);
		goto err_adapter;
	}

	a->demux.priv = a;
	a->demux.filternum = 256;
	a->demux.feednum = 256;
	a->demux.start_feed = idl4k_start_feed;
	a->demux.stop_feed = idl4k_stop_feed;
	a->demux.dmx.capabilities = DMX_TS_FILTERING | DMX_SECTION_FILTERING |
				    DMX_MEMORY_BASED_FILTERING;
	err = dvb_dmx_init(&a->demux);
	if (err < 0)
		goto err_input;

	a->dmxdev.filternum = 256;
	a->dmxdev.demux = &a->demux.dmx;
	a->dmxdev.capabilities = 0;
	err = dvb_dmxdev_init(&a->dmxdev, &a->adapter);
	if (err < 0)
		goto err_dmx;

	dmx = &a->demux.dmx;
	a->hw_frontend.source = DMX_FRONTEND_0;
	err = dmx->add_frontend(dmx, &a->hw_frontend);
	if (err < 0)
		goto err_dmxdev;
	a->mem_frontend.source = DMX_MEMORY_FE;
	err = dmx->add_frontend(dmx, &a->mem_frontend);
	if (err < 0)
		goto err_hw;
	err = dmx->connect_frontend(dmx, &a->hw_frontend);
	if (err < 0)
		goto err_mem;
	err = idl4k_frontend_attach(a);
	if (err < 0)
		goto err_mem;
	return a;

err_mem:
	dmx->remove_frontend(dmx, &a->mem_frontend);
err_hw:
	dmx->remove_frontend(dmx, &a->hw_frontend);
err_dmxdev:
	dvb_dmxdev_release(&a->dmxdev);
err_dmx:
	dvb_dmx_release(&a->demux);
err_input:
	tango_pti_input_put(a->input);
err_adapter:
	dvb_unregister_adapter(&a->adapter);
err_led:
	led_trigger_unregister_simple(a->led);
	kfree(a);
	return ERR_PTR(err);
}

static void idl4k_demod_reset_release(int n)
{
	while (n--)
		gpio_free(idl4k_demod_reset[n]);
}

/* reset both demodulators once, before the first access */
static int idl4k_demod_reset_pulse(void)
{
	int n, err;

	for (n = 0; n < ARRAY_SIZE(idl4k_demod_reset); n++) {
		err = gpio_request(idl4k_demod_reset[n], "demod reset");
		if (err < 0) {
			idl4k_demod_reset_release(n);
			return err;
		}
		gpio_direction_output(idl4k_demod_reset[n], 0);
	}
	msleep(20);
	for (n = 0; n < ARRAY_SIZE(idl4k_demod_reset); n++)
		gpio_set_value(idl4k_demod_reset[n], 1);
	msleep(20);
	return 0;
}

static int __init idl4k_dvb_init(void)
{
	int n, err;

	idl4k_i2c = i2c_get_adapter(IDL4K_I2C_BUS);
	if (!idl4k_i2c)
		return -ENODEV;
	err = idl4k_demod_reset_pulse();
	if (err < 0) {
		i2c_put_adapter(idl4k_i2c);
		return err;
	}

	for (n = 0; n < IDL4K_INPUTS; n++) {
		adapters[n] = idl4k_adapter_add(n);
		if (IS_ERR(adapters[n])) {
			err = PTR_ERR(adapters[n]);
			adapters[n] = NULL;
			while (n--)
				idl4k_adapter_remove(adapters[n]);
			idl4k_demod_reset_release(ARRAY_SIZE(idl4k_demod_reset));
			i2c_put_adapter(idl4k_i2c);
			return err;
		}
	}
	return 0;
}

static void __exit idl4k_dvb_exit(void)
{
	int n;

	for (n = IDL4K_INPUTS - 1; n >= 0; n--)
		if (adapters[n])
			idl4k_adapter_remove(adapters[n]);
	idl4k_demod_reset_release(ARRAY_SIZE(idl4k_demod_reset));
	i2c_put_adapter(idl4k_i2c);
}

module_init(idl4k_dvb_init);
module_exit(idl4k_dvb_exit);
MODULE_DESCRIPTION("Inverto IDL4K DVB adapters");
MODULE_LICENSE("GPL");
