/*
 * TANGO TP and STFE driver, private definitions.
 *
 * The TP side follows the Transport Engine HAL (TE 15.2.2, GPLv2) and talks
 * to the TE firmware through its interface header pti_tp_api.h.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2.
 */
#ifndef __TANGO_PTI_PRIV_H
#define __TANGO_PTI_PRIV_H

#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/irqreturn.h>
#include <linux/seq_file.h>
#include <media/tango_pti.h>

#include "pti_tp_api.h"

/* Resources given to the firmware (TE stptiHAL_pDeviceSetInitialisationParameters) */
#define TP_NUM_VDEVICES		16
#define TP_NUM_SLOTS		256
#define TP_NUM_PIDS		(256 + 128)
#define TP_NUM_SECTION_FILTERS	256
#define TP_NUM_DMA		96
#define TP_NUM_INDEXERS		32
#define TP_NUM_STATUS_BLKS	32

/* Per input: vDevice n, slots n * 64 .. n * 64 + 63 (the last one is the
 * whole-stream slot), PID table entries n * 64 .. n * 64 + 62, DMA record n */
#define INPUT_SLOTS		64
#define INPUT_PIDS		(INPUT_SLOTS - 1)

#define TS_PKT_LEN		188
#define STFE_DMA_PKT_LEN	200	/* 8 tag bytes + 188 + 4 */
#define STFE_RING_PKTS		256
#define OUT_BUF_PKTS		2048

#define TP_INVALID_PID		0xe000

struct tango_pti;

struct tango_pti_input {
	struct tango_pti *pti;
	int index;
	int ib;				/* STFE input block */
	tango_pti_data_fn fn;
	void *priv;
	int in_use;
	int running;

	/* STFE → TP ring (uncached) and TP output buffer (cached) */
	u8 *ring;
	dma_addr_t ring_phys;
	u8 *buf;
	dma_addr_t buf_phys;
	u32 read_offset;

	u16 pid[INPUT_PIDS];		/* TP_INVALID_PID when free */
	u16 users[INPUT_PIDS];
	int all_users;			/* references on TANGO_PTI_ALL_PIDS */

	unsigned long packets;
	u32 max_fill;			/* most packets seen waiting in buf */

	/* STFE IB status, summed by the data thread (it clears on read) */
	int lock;
	unsigned long fifo_overflows;
	unsigned long buffer_overflows;
	unsigned long pkt_overflows;
	unsigned long error_pkts;
	unsigned long short_pkts;
};

struct tango_pti {
	struct device *dev;
	struct tango_pti_platform_data *pdata;
	void __iomem *tp;
	void __iomem *stfe;
	void __iomem *stfe_ram;
	int irq;
	struct clk *clk;
	struct stm_pad_state *pads;

	struct mutex lock;		/* configuration, mailbox, data thread */
	struct completion ack;
	struct task_struct *thread;
	char fw_version[9];

	/* TP shared memory (host addresses inside the TP mapping) */
	volatile stptiTP_Live_t *live;
	volatile stptiTP_pDeviceInfo_t *pdev_info;
	volatile stptiTP_vDeviceInfo_t *vdev_info;
	volatile stptiTP_SlotInfo_t *slot_info;
	volatile stptiTP_DMAInfo_t *dma_info;
	volatile u8 *dma_overflow;
	volatile u16 *pid_table;
	volatile u16 *pid_slot_map;
	volatile u32 *dma_pointers;

	/* STFE internal RAM layout */
	u32 fifo_base;
	u32 fifo_size;

	struct tango_pti_input input[TANGO_PTI_MAX_INPUTS];
};

/* Write/read one TP shared-memory field with an access of its own size */
#define TP_WRITE(lval, v) do {						\
	void __iomem *a_ = (void __iomem *)&(lval);			\
	if (sizeof(lval) == 1)						\
		writeb((u8)(v), a_);					\
	else if (sizeof(lval) == 2)					\
		writew((u16)(v), a_);					\
	else								\
		writel((u32)(v), a_);					\
} while (0)
#define TP_READ(lval) (sizeof(lval) == 1 ? readb((void __iomem *)&(lval)) :	\
		       sizeof(lval) == 2 ? readw((void __iomem *)&(lval)) :	\
		       readl((void __iomem *)&(lval)))

/* tango_tp.c */
int tango_tp_boot(struct tango_pti *pti);
void tango_tp_halt(struct tango_pti *pti);
irqreturn_t tango_tp_irq(int irq, void *dev);
void tango_tp_input_setup(struct tango_pti_input *in);
int tango_tp_live(struct tango_pti_input *in, int start);
void tango_tp_set_pid(struct tango_pti_input *in, int k, u16 pid);
void tango_tp_set_all(struct tango_pti_input *in, int on);
void tango_tp_slot_sync(struct tango_pti *pti);
void tango_tp_show(struct tango_pti *pti, struct seq_file *m);
void tango_tp_input_show(struct tango_pti_input *in, struct seq_file *m);

/* tango_stfe.c */
void tango_stfe_init(struct tango_pti *pti);
void tango_stfe_input_configure(struct tango_pti_input *in);
void tango_stfe_input_enable(struct tango_pti_input *in, int on);
void tango_stfe_input_status(struct tango_pti_input *in);

#endif
