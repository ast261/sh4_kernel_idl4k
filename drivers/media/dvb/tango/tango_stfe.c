/*
 * STFE v1 (STx7108) live inputs: one input block (IB) per TS input, its
 * MEMDMA channel writing 200-byte packets into a DDR ring read by the TP.
 *
 * Register layout and sequences follow the Transport Engine HAL (TE 15.2.2,
 * GPLv2): stfe/pti_tsinput.c (StfeConfigureInput, StfeBlockConfigure,
 * StfeConfigureTP), stfe/pti_stfe.c, stfe/pti_stfe_v1.c.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2.
 */
#include <linux/kernel.h>
#include <linux/io.h>

#include "tango_pti_priv.h"

/* STx7108 STFE */
#define NUM_IBS			6
#define NUM_MIBS		6
#define NUM_SWTS		4
#define STFE_RAM_SIZE		0x4000
#define IB_OFFSET		0x0000
#define SYSTEM_OFFSET		0x0a00
#define MEMDMA_OFFSET		0x0c00
#define CCSC_OFFSET		0x1400

#define PID_BIT_OFFSET		40
#define PID_BIT_SIZE		13

struct ib_regs {
	u32 input_format_config;
	u32 slad_config;
	u32 tag_bytes;
	u32 pid_setup;
	u32 packet_length;
	u32 buffer_start;
	u32 buffer_end;
	u32 read_pointer;
	u32 write_pointer;
	u32 priority_threshold;
	u32 status;
	u32 mask;
	u32 system;
	u8 padding[12];
};

/* in STFE RAM, one per IB and merged IB */
struct input_node {
	u32 bus_base;
	u32 bus_top;
	u32 bus_wp;
	u32 ts_pkt_size;
	u32 bus_next_base;
	u32 bus_next_top;
	u32 mem_base;
	u32 mem_top;
};

struct system_regs {
	u32 status;
	u32 mask;
	u32 dma_route;
	u32 clk_enable;
};

struct memdma_regs {
	u32 data_int_status;
	u32 data_int_mask;
	u32 error_int_status;
	u32 error_int_mask;
	u32 error_override;
	u32 dma_abort;
	u32 message_boundary;
	u32 idle_int_status;
	u32 idle_int_mask;
	u32 dma_mode;
	u8 padding0[0x18];
	u32 pointer_base;
	u8 padding1[0x1bc];
	u32 bus_size[16];
	u32 bus_level[16];
	u32 bus_thres[16];
};

#define SYSTEM_ENABLE		0x00000001
#define SYSTEM_RESET		0x00000002
#define TAG_ENABLE		0x00000001
#define TAG_COUNTER_MASK	0x00000006
#define TAG_HEADER_MASK		0xffff0000
#define SLD_MASK		0x0000ffff
#define PID_ENABLE		0x80000000
#define CCSC_BYPASS		0x00010001

#define IB(s, i)	((struct ib_regs __iomem *)((s)->stfe + IB_OFFSET) + (i))
#define SYS(s)		((struct system_regs __iomem *)((s)->stfe + SYSTEM_OFFSET))
#define MEMDMA(s)	((struct memdma_regs __iomem *)((s)->stfe + MEMDMA_OFFSET))
#define NODE(s, i)	((struct input_node __iomem *)(s)->stfe_ram + (i))

static inline void rmw(u32 __iomem *reg, u32 mask, u32 val)
{
	writel((readl(reg) & ~mask) | (val & mask), reg);
}

/* stptiTSHAL_StfeStartHW, stptiTSHAL_Stfev1_CreateConfig */
void tango_stfe_init(struct tango_pti *pti)
{
	u32 ptrs = (NUM_IBS + NUM_MIBS) * sizeof(struct input_node);

	ptrs = ALIGN(ptrs, 0x40);
	pti->fifo_base = ptrs;
	pti->fifo_size = ((STFE_RAM_SIZE - ptrs) / (NUM_IBS + NUM_MIBS + NUM_SWTS)) & ~0x3f;

	writel(0, &MEMDMA(pti)->pointer_base);
	writel(0x2, pti->stfe + CCSC_OFFSET + 8);	/* CCSC_CLK_CTRL */
	writel(CCSC_BYPASS, pti->stfe + CCSC_OFFSET);	/* CCSC_CTRL */
}

/* stptiTSHAL_Stfev1_Write_RamInputNode_ptrs */
static void write_node(struct tango_pti_input *in)
{
	struct tango_pti *pti = in->pti;
	struct input_node __iomem *n = NODE(pti, in->ib);
	u32 base = in->ring_phys;
	u32 top = base + STFE_RING_PKTS * STFE_DMA_PKT_LEN - 1;
	u32 mbase = pti->fifo_base + in->ib * pti->fifo_size;

	writel(base, &n->bus_base);
	writel(top, &n->bus_top);
	writel(base, &n->bus_wp);
	writel(base, &n->bus_next_base);
	writel(top, &n->bus_next_top);
	writel(STFE_DMA_PKT_LEN, &n->ts_pkt_size);
	writel(mbase, &n->mem_base);
	writel(mbase + pti->fifo_size - 1, &n->mem_top);
}

/*
 * Full IB and MEMDMA set-up, done on every start: restarting the live
 * channel without it leaves the IB in buffer overflow.  The IB is left
 * disabled; tango_stfe_input_enable() starts it once the TP has the ring.
 */
void tango_stfe_input_configure(struct tango_pti_input *in)
{
	struct tango_pti *pti = in->pti;
	struct ib_regs __iomem *ib = IB(pti, in->ib);
	struct memdma_regs __iomem *dma = MEMDMA(pti);
	u32 fifo = pti->fifo_base + in->ib * pti->fifo_size;

	tango_stfe_input_enable(in, 0);
	rmw(&ib->slad_config, SLD_MASK, 3 | (3 << 4) | (0x47 << 8));
	rmw(&ib->tag_bytes, TAG_ENABLE | TAG_HEADER_MASK, TAG_ENABLE | ((u32)in->ib << 16));
	writel(fifo, &ib->buffer_start);
	writel(fifo + pti->fifo_size - 1, &ib->buffer_end);
	/* PID filtering is done by the TP; the IB passes every packet */
	writel(TS_PKT_LEN, &ib->packet_length);
	writel((PID_BIT_SIZE << 6) | (PID_BIT_OFFSET & 0x3f), &ib->pid_setup);
	writel(pti->pdata->input_format, &ib->input_format_config);
	rmw(&ib->tag_bytes, TAG_COUNTER_MASK, 0);
	rmw(&SYS(pti)->dma_route, 1 << in->ib, 0);	/* to the TP */

	/* MemdmaClearDREQLevel and the DDR pointers */
	rmw(&dma->dma_mode, 1 << in->ib, 1 << in->ib);
	writel(STFE_RING_PKTS, &dma->bus_size[in->ib]);
	writel(0, &dma->bus_level[in->ib]);
	writel(0x00010000 << in->ib, &dma->error_int_status);
	write_node(in);
}

/* stptiTSHAL_StfeEnable */
void tango_stfe_input_enable(struct tango_pti_input *in, int on)
{
	struct tango_pti *pti = in->pti;
	struct ib_regs __iomem *ib = IB(pti, in->ib);

	readl(&ib->status);
	writel(on ? 0xffffffff : 0, &ib->mask);
	writel(on ? (SYSTEM_ENABLE | SYSTEM_RESET) : 0, &ib->system);
	if (!on)
		rmw(&MEMDMA(pti)->dma_mode, 1 << in->ib, 0);
}
