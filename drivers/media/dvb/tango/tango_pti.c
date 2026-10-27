/*
 * STx7108 TANGO transport processor and STFE live inputs.
 *
 * The TP runs the Transport Engine firmware (pti_tp2.elf on the 7108).
 * Each input gets its own vDevice, a RAW slot per PID and one output
 * buffer; a kernel thread hands the filtered packets to the client.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/stm/pad.h>

#include "tango_pti_priv.h"

#define POLL_MS		10

static struct tango_pti *tango_pti;

/* Deliver what the TP wrote since the last call; called with pti->lock held */
static void input_poll(struct tango_pti_input *in)
{
	const u32 size = OUT_BUF_PKTS * TS_PKT_LEN;
	u32 q = TP_READ(in->pti->dma_info[in->index].qwrite_offset);
	u32 r = in->read_offset;
	u32 n;

	if (q >= size || q % TS_PKT_LEN)
		return;
	/* the buffer is cached: drop stale lines before reading what the TP wrote */
	if (q < r) {
		n = (size - r) / TS_PKT_LEN;
		dma_cache_sync(in->pti->dev, in->buf + r, size - r, DMA_FROM_DEVICE);
		in->fn(in->priv, in->buf + r, n);
		in->packets += n;
		r = 0;
	}
	if (q > r) {
		n = (q - r) / TS_PKT_LEN;
		dma_cache_sync(in->pti->dev, in->buf + r, q - r, DMA_FROM_DEVICE);
		in->fn(in->priv, in->buf + r, n);
		in->packets += n;
		r = q;
	}
	in->read_offset = r;
	TP_WRITE(in->pti->dma_info[in->index].read_offset, r);
}

static int tango_pti_thread(void *data)
{
	struct tango_pti *pti = data;
	int i;

	while (!kthread_should_stop()) {
		msleep(POLL_MS);
		mutex_lock(&pti->lock);
		for (i = 0; i < pti->pdata->num_inputs; i++)
			if (pti->input[i].running)
				input_poll(&pti->input[i]);
		mutex_unlock(&pti->lock);
	}
	return 0;
}

/* TE order: STFE pointers, TP live channel, then the IB */
static int input_start(struct tango_pti_input *in)
{
	int err;

	tango_tp_input_setup(in);
	tango_stfe_input_configure(in);
	err = tango_tp_live(in, 1);
	if (err) {
		dev_err(in->pti->dev, "input %d: TP did not start the live channel\n", in->index);
		return err;
	}
	tango_stfe_input_enable(in, 1);
	in->running = 1;
	return 0;
}

static void input_stop(struct tango_pti_input *in)
{
	tango_stfe_input_enable(in, 0);
	if (tango_tp_live(in, 0))
		dev_err(in->pti->dev, "input %d: TP did not stop the live channel\n", in->index);
	in->running = 0;
}

static int input_has_pids(struct tango_pti_input *in)
{
	int k;

	if (in->all_users)
		return 1;
	for (k = 0; k < INPUT_PIDS; k++)
		if (in->users[k])
			return 1;
	return 0;
}

struct tango_pti_input *tango_pti_input_get(unsigned int n, tango_pti_data_fn fn, void *priv)
{
	struct tango_pti *pti = tango_pti;
	struct tango_pti_input *in;

	if (!pti || n >= pti->pdata->num_inputs)
		return ERR_PTR(-ENODEV);
	in = &pti->input[n];
	mutex_lock(&pti->lock);
	if (in->in_use) {
		mutex_unlock(&pti->lock);
		return ERR_PTR(-EBUSY);
	}
	in->in_use = 1;
	in->fn = fn;
	in->priv = priv;
	mutex_unlock(&pti->lock);
	return in;
}
EXPORT_SYMBOL_GPL(tango_pti_input_get);

void tango_pti_input_put(struct tango_pti_input *in)
{
	struct tango_pti *pti = in->pti;
	int k;

	mutex_lock(&pti->lock);
	if (in->running)
		input_stop(in);
	for (k = 0; k < INPUT_PIDS; k++) {
		in->pid[k] = TP_INVALID_PID;
		in->users[k] = 0;
	}
	in->all_users = 0;
	in->in_use = 0;
	mutex_unlock(&pti->lock);
}
EXPORT_SYMBOL_GPL(tango_pti_input_put);

int tango_pti_pid_add(struct tango_pti_input *in, u16 pid)
{
	struct tango_pti *pti = in->pti;
	int k, free = -1, err = 0;

	if (pid > TANGO_PTI_ALL_PIDS)
		return -EINVAL;
	mutex_lock(&pti->lock);
	if (!in->running) {
		err = input_start(in);
		if (err)
			goto out;
	}
	if (pid == TANGO_PTI_ALL_PIDS) {
		if (!in->all_users++)
			tango_tp_set_all(in, 1);
		goto out;
	}
	for (k = 0; k < INPUT_PIDS; k++) {
		if (in->users[k] && in->pid[k] == pid) {
			in->users[k]++;
			goto out;
		}
		if (!in->users[k] && free < 0)
			free = k;
	}
	if (free < 0) {
		err = -ENOSPC;
		goto out;
	}
	in->pid[free] = pid;
	in->users[free] = 1;
	tango_tp_set_pid(in, free, pid);
out:
	if (err && in->running && !input_has_pids(in))
		input_stop(in);
	mutex_unlock(&pti->lock);
	return err;
}
EXPORT_SYMBOL_GPL(tango_pti_pid_add);

void tango_pti_pid_remove(struct tango_pti_input *in, u16 pid)
{
	struct tango_pti *pti = in->pti;
	int k;

	mutex_lock(&pti->lock);
	if (pid == TANGO_PTI_ALL_PIDS) {
		if (in->all_users && !--in->all_users)
			tango_tp_set_all(in, 0);
	} else {
		for (k = 0; k < INPUT_PIDS; k++) {
			if (in->users[k] && in->pid[k] == pid) {
				if (!--in->users[k]) {
					tango_tp_set_pid(in, k, TP_INVALID_PID);
					in->pid[k] = TP_INVALID_PID;
				}
				break;
			}
		}
	}
	if (in->running && !input_has_pids(in))
		input_stop(in);
	mutex_unlock(&pti->lock);
}
EXPORT_SYMBOL_GPL(tango_pti_pid_remove);

static void free_buffers(struct tango_pti *pti)
{
	int i;

	for (i = 0; i < pti->pdata->num_inputs; i++) {
		struct tango_pti_input *in = &pti->input[i];

		if (in->buf) {
			dma_unmap_single(pti->dev, in->buf_phys, OUT_BUF_PKTS * TS_PKT_LEN,
					 DMA_FROM_DEVICE);
			free_pages_exact(in->buf, OUT_BUF_PKTS * TS_PKT_LEN);
		}
		if (in->ring)
			dma_free_coherent(pti->dev, STFE_RING_PKTS * STFE_DMA_PKT_LEN,
					  in->ring, in->ring_phys);
	}
}

static void __iomem *map_resource(struct platform_device *pdev, const char *name)
{
	struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);

	if (!res || !request_mem_region(res->start, resource_size(res), name))
		return NULL;
	return ioremap_nocache(res->start, resource_size(res));
}

static void unmap_resource(struct platform_device *pdev, const char *name, void __iomem *p)
{
	struct resource *res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);

	if (p)
		iounmap(p);
	if (res && p)
		release_mem_region(res->start, resource_size(res));
}

static void unmap_all(struct platform_device *pdev, struct tango_pti *pti)
{
	unmap_resource(pdev, "tp", pti->tp);
	unmap_resource(pdev, "stfe", pti->stfe);
	unmap_resource(pdev, "stfe-ram", pti->stfe_ram);
}

static int __devinit tango_pti_probe(struct platform_device *pdev)
{
	struct tango_pti_platform_data *pdata = pdev->dev.platform_data;
	struct tango_pti *pti;
	int i, err;

	if (!pdata || pdata->num_inputs < 1 || pdata->num_inputs > TANGO_PTI_MAX_INPUTS || tango_pti)
		return -EINVAL;
	pti = kzalloc(sizeof(*pti), GFP_KERNEL);
	if (!pti)
		return -ENOMEM;
	pti->dev = &pdev->dev;
	pti->pdata = pdata;
	mutex_init(&pti->lock);
	init_completion(&pti->ack);

	pti->tp = map_resource(pdev, "tp");
	pti->stfe = map_resource(pdev, "stfe");
	pti->stfe_ram = map_resource(pdev, "stfe-ram");
	pti->irq = platform_get_irq_byname(pdev, "tp");
	if (!pti->tp || !pti->stfe || !pti->stfe_ram || pti->irq < 0) {
		err = -ENXIO;
		goto err_unmap;
	}

	for (i = 0; i < pdata->num_inputs; i++) {
		struct tango_pti_input *in = &pti->input[i];
		int k;

		in->pti = pti;
		in->index = i;
		in->ib = pdata->input_ib[i];
		for (k = 0; k < INPUT_PIDS; k++)
			in->pid[k] = TP_INVALID_PID;
		in->ring = dma_alloc_coherent(&pdev->dev, STFE_RING_PKTS * STFE_DMA_PKT_LEN,
					      &in->ring_phys, GFP_KERNEL);
		/* cached, so that the demux reads it at memory speed */
		in->buf = alloc_pages_exact(OUT_BUF_PKTS * TS_PKT_LEN, GFP_KERNEL);
		if (!in->ring || !in->buf) {
			err = -ENOMEM;
			goto err_free;
		}
		in->buf_phys = dma_map_single(&pdev->dev, in->buf, OUT_BUF_PKTS * TS_PKT_LEN,
					      DMA_FROM_DEVICE);
	}

	if (pdata->pad_config) {
		pti->pads = stm_pad_claim(pdata->pad_config, dev_name(&pdev->dev));
		if (!pti->pads) {
			dev_err(&pdev->dev, "cannot claim the TSIN pads\n");
			err = -EBUSY;
			goto err_free;
		}
	}

	pti->clk = clk_get(&pdev->dev, pdata->clk_name);
	if (IS_ERR(pti->clk)) {
		err = PTR_ERR(pti->clk);
		goto err_pads;
	}
	clk_enable(pti->clk);

	err = request_irq(pti->irq, tango_tp_irq, 0, dev_name(&pdev->dev), pti);
	if (err)
		goto err_clk;

	err = tango_tp_boot(pti);
	if (err)
		goto err_irq;
	tango_stfe_init(pti);

	pti->thread = kthread_run(tango_pti_thread, pti, "tango_pti");
	if (IS_ERR(pti->thread)) {
		err = PTR_ERR(pti->thread);
		goto err_halt;
	}

	platform_set_drvdata(pdev, pti);
	tango_pti = pti;
	dev_info(&pdev->dev, "%d live inputs\n", pdata->num_inputs);
	return 0;

err_halt:
	tango_tp_halt(pti);
err_irq:
	free_irq(pti->irq, pti);
err_clk:
	clk_disable(pti->clk);
	clk_put(pti->clk);
err_pads:
	if (pti->pads)
		stm_pad_release(pti->pads);
err_free:
	free_buffers(pti);
err_unmap:
	unmap_all(pdev, pti);
	kfree(pti);
	return err;
}

static int __devexit tango_pti_remove(struct platform_device *pdev)
{
	struct tango_pti *pti = platform_get_drvdata(pdev);
	int i;

	kthread_stop(pti->thread);
	mutex_lock(&pti->lock);
	for (i = 0; i < pti->pdata->num_inputs; i++)
		if (pti->input[i].running)
			input_stop(&pti->input[i]);
	mutex_unlock(&pti->lock);
	tango_pti = NULL;
	tango_tp_halt(pti);
	free_irq(pti->irq, pti);
	clk_disable(pti->clk);
	clk_put(pti->clk);
	if (pti->pads)
		stm_pad_release(pti->pads);
	free_buffers(pti);
	unmap_all(pdev, pti);
	kfree(pti);
	return 0;
}

static struct platform_driver tango_pti_driver = {
	.probe = tango_pti_probe,
	.remove = __devexit_p(tango_pti_remove),
	.driver = {
		.name = "tango-pti",
		.owner = THIS_MODULE,
	},
};

static int __init tango_pti_init(void)
{
	return platform_driver_register(&tango_pti_driver);
}

static void __exit tango_pti_exit(void)
{
	platform_driver_unregister(&tango_pti_driver);
}

module_init(tango_pti_init);
module_exit(tango_pti_exit);
MODULE_DESCRIPTION("STx7108 TANGO TP and STFE live inputs");
MODULE_LICENSE("GPL");
