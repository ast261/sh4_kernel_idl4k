/*
 * Inverto IDL4K: one DVB adapter per satellite input.
 *
 * The demux of each adapter programs PID filters on the TANGO transport
 * processor (tango-pti) and gets the filtered packets back into the
 * software demux.  The frontends are not attached here yet.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <media/tango_pti.h>

#include "dvbdev.h"
#include "dvb_demux.h"
#include "dmxdev.h"

#define IDL4K_INPUTS	4

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

struct idl4k_adapter {
	int index;
	struct dvb_adapter adapter;
	struct dvb_demux demux;
	struct dmxdev dmxdev;
	struct dmx_frontend hw_frontend;
	struct dmx_frontend mem_frontend;
	struct tango_pti_input *input;
};

static struct idl4k_adapter *adapters[IDL4K_INPUTS];

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

static void idl4k_adapter_remove(struct idl4k_adapter *a)
{
	struct dmx_demux *dmx = &a->demux.dmx;

	dmx->remove_frontend(dmx, &a->mem_frontend);
	dmx->remove_frontend(dmx, &a->hw_frontend);
	dvb_dmxdev_release(&a->dmxdev);
	dvb_dmx_release(&a->demux);
	tango_pti_input_put(a->input);
	dvb_unregister_adapter(&a->adapter);
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

	err = dvb_register_adapter(&a->adapter, "IDL4K", THIS_MODULE, NULL, adapter_nr);
	if (err < 0)
		goto err_free;

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
err_free:
	kfree(a);
	return ERR_PTR(err);
}

static int __init idl4k_dvb_init(void)
{
	int n;

	for (n = 0; n < IDL4K_INPUTS; n++) {
		adapters[n] = idl4k_adapter_add(n);
		if (IS_ERR(adapters[n])) {
			int err = PTR_ERR(adapters[n]);

			adapters[n] = NULL;
			while (n--)
				idl4k_adapter_remove(adapters[n]);
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
}

module_init(idl4k_dvb_init);
module_exit(idl4k_dvb_exit);
MODULE_DESCRIPTION("Inverto IDL4K DVB adapters");
MODULE_LICENSE("GPL");
