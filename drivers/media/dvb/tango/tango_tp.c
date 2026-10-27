/*
 * TANGO transport processor (STxP70) running the Transport Engine firmware:
 * boot, mailbox, and the shared-memory tables for live PID filtering.
 *
 * Register layout and sequences follow the Transport Engine HAL (TE 15.2.2,
 * GPLv2): tango/pti_pdevice.c, pti_mailbox.c, pti_vdevice.c, pti_slot.c,
 * pti_buffer.c.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2.
 */
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/device.h>

#include "tango_pti_priv.h"

/* TP memory map */
#define DDEM_OFF		0x00000
#define IDEM_OFF		0x10000
#define CORE_OFF		0x2d000
#define MBX_TO_TP		0x2d020
#define MBX_TO_HOST		0x2d040

/* core control */
#define RESET_CTRL		0x00
#define RESET_STATUS		0x04
#define CORE_RUN_CFG		0x0c
#define PWRDN_CFG		0x14
/* mailbox */
#define MBX_STATUS		0x00
#define MBX_SET			0x04
#define MBX_CLEAR		0x08
#define MBX_MASK		0x0c

#define IF_OFF(f)		(DDEM_OFF + offsetof(stptiTP_Interface_t, f))
#define IF_READ(pti, f)		readl((pti)->tp + IF_OFF(f))
#define IF_WRITE(pti, f, v)	writel((v), (pti)->tp + IF_OFF(f))
/* the interface pointers are dDEM offsets on the TP */
#define IF_PTR(pti, f)		((void *)((pti)->tp + DDEM_OFF + IF_READ(pti, f)))

static void tp_memset0(volatile void *p, size_t n)
{
	size_t i;

	for (i = 0; i + 4 <= n; i += 4)
		writel(0, (void __iomem *)p + i);
	for (; i < n; i++)
		writeb(0, (void __iomem *)p + i);
}

static void tp_printf(struct tango_pti *pti)
{
	char buf[257];
	int i;

	for (i = 0; i < 256; i += 4)
		*(u32 *)&buf[i] = readl(pti->tp + IF_OFF(DEBUG_BUFFER) + i);
	buf[256] = 0;
	dev_info(pti->dev, "TP: %s\n", strstrip(buf));
	writel(stptiTP_H2TP_MAILBOX0_TP_PRINTF_COMPLETE_MASK, pti->tp + MBX_TO_TP + MBX_SET);
}

irqreturn_t tango_tp_irq(int irq, void *dev)
{
	struct tango_pti *pti = dev;
	u32 st = readl(pti->tp + MBX_TO_HOST + MBX_STATUS) &
		 readl(pti->tp + MBX_TO_HOST + MBX_MASK);

	if (!st)
		return IRQ_NONE;
	writel(st, pti->tp + MBX_TO_HOST + MBX_CLEAR);
	if (st & stptiTP_TP2H_MAILBOX0_IS_TP_PRINTF_WAITING_MASK)
		tp_printf(pti);
	if (st & stptiTP_TP2H_MAILBOX0_TP_ACKNOWLEDGE_MASK)
		complete(&pti->ack);
	return IRQ_HANDLED;
}

/* Post a host-to-TP mailbox bit and wait for the TP's acknowledge */
static int tp_post(struct tango_pti *pti, u32 mask, int ms)
{
	INIT_COMPLETION(pti->ack);
	writel(mask, pti->tp + MBX_TO_TP + MBX_SET);
	if (!wait_for_completion_timeout(&pti->ack, msecs_to_jiffies(ms)))
		return -ETIMEDOUT;
	return 0;
}

static void tp_core_reset(struct tango_pti *pti)
{
	int t;

	writel(0, pti->tp + CORE_OFF + CORE_RUN_CFG);
	writel(1, pti->tp + CORE_OFF + RESET_CTRL);
	for (t = 0; t < 1000 && !(readl(pti->tp + CORE_OFF + RESET_STATUS) & 1); t++)
		udelay(10);
}

/* Data (dDEM) and code (iDEM) segments of the firmware ELF */
static int tp_load_elf(struct tango_pti *pti, const u8 *data, size_t size)
{
	const Elf32_Ehdr *eh = (const Elf32_Ehdr *)data;
	const Elf32_Phdr *ph;
	int i;

	if (size < sizeof(*eh) || memcmp(eh->e_ident, ELFMAG, SELFMAG) ||
	    eh->e_type != ET_EXEC || eh->e_phoff + eh->e_phnum * sizeof(*ph) > size)
		return -EINVAL;
	ph = (const Elf32_Phdr *)(data + eh->e_phoff);
	for (i = 0; i < eh->e_phnum; i++, ph++) {
		void __iomem *dst;
		u32 o;

		if (ph->p_type != PT_LOAD)
			continue;
		if (ph->p_offset + ph->p_filesz > size || ph->p_filesz > ph->p_memsz)
			return -EINVAL;
		if (ph->p_paddr < 0x00400000)
			dst = pti->tp + DDEM_OFF + (ph->p_paddr & 0xfffff);
		else
			dst = pti->tp + IDEM_OFF + (ph->p_paddr & 0xfffff);
		for (o = 0; o < ph->p_filesz; o += 4) {
			u32 w = 0;

			memcpy(&w, data + ph->p_offset + o, min_t(u32, 4, ph->p_filesz - o));
			writel(w, dst + o);
		}
		for (; o < ph->p_memsz; o += 4)
			writel(0, dst + o);
	}
	return 0;
}

/* TE stptiHAL_pDeviceInit: the structure resets done after the init ack */
static void tp_tables_init(struct tango_pti *pti)
{
	u32 nvdev = IF_READ(pti, NumberOfvDevices);
	u32 nslots = IF_READ(pti, NumberOfSlots);
	u32 ndma = IF_READ(pti, NumberOfDMAStructures);
	u32 npids = IF_READ(pti, NumberOfPids);
	u32 i;

	pti->live = IF_PTR(pti, Live_p);
	pti->pdev_info = IF_PTR(pti, pDeviceInfo_p);
	pti->vdev_info = IF_PTR(pti, vDeviceInfo_p);
	pti->slot_info = IF_PTR(pti, SlotInfo_p);
	pti->dma_info = IF_PTR(pti, DMAInfo_p);
	pti->dma_overflow = IF_PTR(pti, DMAOverflowFlags_p);
	pti->pid_table = IF_PTR(pti, PIDTable_p);
	pti->pid_slot_map = IF_PTR(pti, PIDSlotMappingTable_p);
	pti->dma_pointers = IF_PTR(pti, DMAPointers_p);

	TP_WRITE(((volatile stptiTP_Interface_t *)(pti->tp + DDEM_OFF))->OutputDebugLevel, 0);
	TP_WRITE(pti->pdev_info->ResetIdleCounters, 1);
	TP_WRITE(pti->pdev_info->SWLeakyPIDChannels, 0);
	for (i = 0; i < nvdev; i++) {
		volatile stptiTP_vDeviceInfo_t *v = &pti->vdev_info[i];

		tp_memset0(v, sizeof(*v));
		TP_WRITE(v->stream_tag, 0xffff);
		TP_WRITE(v->wildcard_slot_index, 0xffff);
		TP_WRITE(v->DataEntry.EntryState, DATA_ENTRY_STATE_EMPTY);
		TP_WRITE(v->DataEntry.EntrySlotIndex, 0xffff);
	}
	for (i = 0; i < nslots; i++) {
		volatile stptiTP_SlotInfo_t *s = &pti->slot_info[i];

		tp_memset0(s, sizeof(*s));
		TP_WRITE(s->slot_state, STATUS_SLOT_STATE_CLEAR | STATUS_TS_LEVEL_SC_NOT_SYNCED);
		TP_WRITE(s->next_slot, 0xffff);
		TP_WRITE(s->dma_record, 0xffff);
		TP_WRITE(s->indexer, 0xffff);
		TP_WRITE(s->event_mask, EVENT_MASK_ALL_EVENTS & ~EVENT_MASK_SLOT_ENABLED_EVENTS);
	}
	for (i = 0; i < ndma; i++) {
		tp_memset0(&pti->dma_info[i], sizeof(pti->dma_info[i]));
		TP_WRITE(pti->dma_info[i].signal_threshold, DMA_INFO_NO_SIGNALLING);
	}
	for (i = 0; i < npids; i++) {
		TP_WRITE(pti->pid_table[i], TP_INVALID_PID);
		TP_WRITE(pti->pid_slot_map[i], 0xffff);
	}
}

/* TE stptiHAL_pDeviceInit / stptiHAL_pDeviceSetInitialisationParameters */
int tango_tp_boot(struct tango_pti *pti)
{
	const struct firmware *fw;
	char ver[9];
	int err;

	err = request_firmware(&fw, pti->pdata->firmware, pti->dev);
	if (err) {
		dev_err(pti->dev, "no firmware %s (%d)\n", pti->pdata->firmware, err);
		return err;
	}

	tp_core_reset(pti);
	writel(0, pti->tp + MBX_TO_HOST + MBX_MASK);
	writel(0xffffffff, pti->tp + MBX_TO_HOST + MBX_CLEAR);
	writel(stptiTP_TP2H_MAILBOX0_IS_TP_PRINTF_WAITING_MASK |
	       stptiTP_TP2H_MAILBOX0_TP_ACKNOWLEDGE_MASK,
	       pti->tp + MBX_TO_HOST + MBX_MASK);

	err = tp_load_elf(pti, fw->data, fw->size);
	release_firmware(fw);
	if (err) {
		dev_err(pti->dev, "bad firmware image %s\n", pti->pdata->firmware);
		return err;
	}

	INIT_COMPLETION(pti->ack);
	IF_WRITE(pti, ActivityCounter, 0xf0000000);
	writel(2, pti->tp + CORE_OFF + PWRDN_CFG);
	writel(1, pti->tp + CORE_OFF + CORE_RUN_CFG);
	if (!wait_for_completion_timeout(&pti->ack, msecs_to_jiffies(2000))) {
		dev_err(pti->dev, "TP did not start\n");
		err = -ETIMEDOUT;
		goto fail;
	}

	*(u32 *)&ver[0] = IF_READ(pti, Version);
	*(u32 *)&ver[4] = readl(pti->tp + IF_OFF(Version) + 4);
	ver[8] = 0;
	strcpy(pti->fw_version, ver);
	/* TE checks the version string; the interface checksum is what matters
	 * for the shared-memory layout (the 7108 images say 01022400) */
	if (IF_READ(pti, CheckSum) != SHARED_MEMORY_INTERFACE_CHECKSUM || strncmp(ver, "0102", 4)) {
		dev_err(pti->dev, "firmware %s checksum 0x%08x not supported\n",
			ver, IF_READ(pti, CheckSum));
		err = -EINVAL;
		goto fail;
	}

	IF_WRITE(pti, NumberOfvDevices, TP_NUM_VDEVICES);
	IF_WRITE(pti, NumberOfSlots, TP_NUM_SLOTS);
	IF_WRITE(pti, NumberOfPids, TP_NUM_PIDS);
	IF_WRITE(pti, NumberOfSectionFilters, TP_NUM_SECTION_FILTERS);
	IF_WRITE(pti, NumberOfDMAStructures, TP_NUM_DMA);
	IF_WRITE(pti, NumberOfIndexers, TP_NUM_INDEXERS);
	IF_WRITE(pti, NumberOfStatusBlks, TP_NUM_STATUS_BLKS);
	IF_WRITE(pti, SlowRateStreamTimeout, 0xffffffff);
	/* the 7108 TP has no timer counter block */
	IF_WRITE(pti, FirmwareConfig, TP_FIRMWARE_RESET_SHARED_MEMORY |
		 TP_FIRMWARE_BYPASS_SECURE_COPRO);
	err = tp_post(pti, stptiTP_H2TP_MAILBOX0_SUPPLYING_INIT_PARAMS_MASK, 100);
	if (err) {
		dev_err(pti->dev, "TP did not take the init parameters\n");
		goto fail;
	}
	if (IF_READ(pti, NumberOfvDevices) < pti->pdata->num_inputs ||
	    IF_READ(pti, NumberOfSlots) < pti->pdata->num_inputs * INPUT_SLOTS ||
	    IF_READ(pti, NumberOfPids) < pti->pdata->num_inputs * INPUT_SLOTS ||
	    IF_READ(pti, NumberOfDMAStructures) < pti->pdata->num_inputs) {
		dev_err(pti->dev, "TP granted too few resources\n");
		err = -ENOMEM;
		goto fail;
	}

	dev_dbg(pti->dev, "shm live 0x%x pdev 0x%x vdev 0x%x pidregion 0x%x pid 0x%x map 0x%x slot 0x%x dma 0x%x ovf 0x%x ptrs 0x%x size %u\n",
		 IF_READ(pti, Live_p), IF_READ(pti, pDeviceInfo_p), IF_READ(pti, vDeviceInfo_p),
		 IF_READ(pti, PIDTableRegion_p), IF_READ(pti, PIDTable_p),
		 IF_READ(pti, PIDSlotMappingTable_p), IF_READ(pti, SlotInfo_p),
		 IF_READ(pti, DMAInfo_p), IF_READ(pti, DMAOverflowFlags_p),
		 IF_READ(pti, DMAPointers_p), IF_READ(pti, SizeOfPIDTableRegion));
	tp_tables_init(pti);
	dev_info(pti->dev, "TP firmware %s: %u vDevices, %u slots, %u PIDs, %u DMA, %u live channels\n",
		 ver, IF_READ(pti, NumberOfvDevices), IF_READ(pti, NumberOfSlots),
		 IF_READ(pti, NumberOfPids), IF_READ(pti, NumberOfDMAStructures),
		 IF_READ(pti, NumberOfLiveChannels));
	return 0;

fail:
	tango_tp_halt(pti);
	return err;
}

void tango_tp_halt(struct tango_pti *pti)
{
	writel(0, pti->tp + MBX_TO_HOST + MBX_MASK);
	tp_core_reset(pti);
}

/* TE stptiHAL_SlotTPSync: wait until the TP is between packets */
void tango_tp_slot_sync(struct tango_pti *pti)
{
	volatile u16 *sync = &pti->pdev_info->SyncTP;
	int t;

	for (t = 0; t < 100 && TP_READ(*sync) != SYNC_TP_SYNCHRONISED; t++)
		udelay(10);
	TP_WRITE(*sync, SYNC_TP_SLOT_WAIT);
	for (t = 0; t < 100 && TP_READ(*sync) != SYNC_TP_SYNCHRONISED; t++)
		udelay(10);
	if (t == 100)
		dev_warn(pti->dev, "no slot sync from the TP\n");
	TP_WRITE(*sync, SYNC_TP_SYNCHRONISED);
}

static void tp_slot_init(struct tango_pti *pti, int slot, int dma)
{
	volatile stptiTP_SlotInfo_t *s = &pti->slot_info[slot];

	/* stptiHAL_SlotTPEntryInitialise, stptiHAL_SlotAllocator (RAW) */
	tp_memset0(s, sizeof(*s));
	TP_WRITE(s->slot_state, STATUS_SLOT_STATE_CLEAR | STATUS_TS_LEVEL_SC_NOT_SYNCED);
	TP_WRITE(s->next_slot, 0xffff);
	TP_WRITE(s->indexer, 0xffff);
	TP_WRITE(s->event_mask, EVENT_MASK_ALL_EVENTS & ~EVENT_MASK_SLOT_ENABLED_EVENTS);
	/* scrambling bits unchanged, no PID remapping */
	TP_WRITE(s->u.raw_slot_info.SCRemappingBits, 0xe4);
	TP_WRITE(s->u.raw_slot_info.RemapPid, TP_INVALID_PID);
	TP_WRITE(s->u.raw_slot_info.DataEntry.EntryState, DATA_ENTRY_STATE_EMPTY);
	/* stptiHAL_SlotAssociator (buffer), stptiHAL_SlotActivate */
	TP_WRITE(s->dma_record, dma);
	TP_WRITE(s->slot_mode, SLOTTYPE_RAW);
}

/*
 * vDevice, slots and output buffer of an input, before the live channel is
 * started (TE stptiHAL_vDeviceAllocator, stptiHAL_BufferDMAConfigure).
 */
void tango_tp_input_setup(struct tango_pti_input *in)
{
	struct tango_pti *pti = in->pti;
	volatile stptiTP_vDeviceInfo_t *v = &pti->vdev_info[in->index];
	volatile stptiTP_DMAInfo_t *d = &pti->dma_info[in->index];
	int base = in->index * INPUT_SLOTS;
	int k;

	for (k = 0; k < INPUT_SLOTS; k++) {
		TP_WRITE(pti->pid_table[base + k], TP_INVALID_PID);
		TP_WRITE(pti->pid_slot_map[base + k], 0xffff);
	}

	tp_memset0(v, sizeof(*v));
	TP_WRITE(v->mode, PACKET_FORMAT_DVB);
	TP_WRITE(v->pid_filter_base, base);
	TP_WRITE(v->pid_filter_size, INPUT_PIDS);
	/* the STFE tags each packet with its IB number */
	TP_WRITE(v->stream_tag, in->ib);
	TP_WRITE(v->wildcard_slot_index, 0xffff);
	TP_WRITE(v->DataEntry.EntryState, DATA_ENTRY_STATE_EMPTY);
	TP_WRITE(v->DataEntry.EntrySlotIndex, 0xffff);
	TP_WRITE(v->event_mask, (EVENT_MASK_ALL_EVENTS & ~EVENT_MASK_VDEVICE_ENABLED_EVENTS) |
		 STATUS_BLK_STATUS_BLK_OVERFLOW);

	/* one buffer for all slots of the input; discard input when full */
	tp_memset0(d, sizeof(*d));
	TP_WRITE(d->base, in->buf_phys);
	TP_WRITE(d->size, OUT_BUF_PKTS * TS_PKT_LEN);
	TP_WRITE(d->signal_threshold, DMA_INFO_NO_SIGNALLING);
	TP_WRITE(pti->dma_overflow[in->index], 0);
	in->read_offset = 0;

	for (k = 0; k < INPUT_SLOTS; k++)
		tp_slot_init(pti, base + k, in->index);
}

/* PID table entry k of the input: slot base + k gets the PID (TE stptiHAL_SlotChainSetPID) */
void tango_tp_set_pid(struct tango_pti_input *in, int k, u16 pid)
{
	struct tango_pti *pti = in->pti;
	int idx = in->index * INPUT_SLOTS + k;

	if (pid == TP_INVALID_PID) {
		TP_WRITE(pti->pid_table[idx], TP_INVALID_PID);
		tango_tp_slot_sync(pti);
		TP_WRITE(pti->pid_slot_map[idx], 0xffff);
		return;
	}
	tp_slot_init(pti, idx, in->index);
	TP_WRITE(pti->pid_slot_map[idx], idx);
	TP_WRITE(pti->pid_table[idx], pid);	/* last */
}

/* The input's last slot takes every packet that matches no PID */
void tango_tp_set_all(struct tango_pti_input *in, int on)
{
	struct tango_pti *pti = in->pti;
	int slot = in->index * INPUT_SLOTS + INPUT_PIDS;

	if (on) {
		tp_slot_init(pti, slot, in->index);
		TP_WRITE(pti->vdev_info[in->index].wildcard_slot_index, slot);
	} else {
		TP_WRITE(pti->vdev_info[in->index].wildcard_slot_index, 0xffff);
		tango_tp_slot_sync(pti);
	}
}

/* TE stptiHAL_pDeviceConfigureLive: hand the STFE ring to the TP or take it back */
int tango_tp_live(struct tango_pti_input *in, int start)
{
	struct tango_pti *pti = in->pti;

	TP_WRITE(pti->live->channel, in->ib);
	TP_WRITE(pti->live->buffer_base, start ? in->ring_phys : 0);
	TP_WRITE(pti->live->buffer_size_in_pkts, start ? STFE_RING_PKTS : 0);
	TP_WRITE(pti->live->packet_len, start ? STFE_DMA_PKT_LEN : 0);
	return tp_post(pti, start ? stptiTP_H2TP_MAILBOX0_START_LIVE_MASK :
		       stptiTP_H2TP_MAILBOX0_STOP_LIVE_MASK, 100);
}

void tango_tp_show(struct tango_pti *pti, struct seq_file *m)
{
	seq_printf(m, "TP firmware %s, activity %u, sync errors %u\n", pti->fw_version,
		   IF_READ(pti, ActivityCounter), TP_READ(pti->pdev_info->PacketsWithSyncError));
}

/* The vDevice and slot counters restart with every start of the input */
void tango_tp_input_show(struct tango_pti_input *in, struct seq_file *m)
{
	struct tango_pti *pti = in->pti;
	volatile stptiTP_vDeviceInfo_t *v = &pti->vdev_info[in->index];
	int base = in->index * INPUT_SLOTS;
	int k;

	seq_printf(m, "  TP: in %u, no PID match %u, sync errors %u, TEI %u, CC errors %u, buffer overflows %u\n",
		   TP_READ(v->input_packet_count), TP_READ(v->pid_mismatch_count),
		   TP_READ(v->transport_sync_error_count), TP_READ(v->transport_error_count),
		   TP_READ(v->cc_error_count), TP_READ(v->buffer_overflow_count));
	seq_printf(m, "  PIDs:");
	for (k = 0; k < INPUT_PIDS; k++)
		if (in->users[k])
			seq_printf(m, " %u:%u", in->pid[k],
				   TP_READ(pti->slot_info[base + k].packet_count));
	if (in->all_users)
		seq_printf(m, " all:%u", TP_READ(pti->slot_info[base + INPUT_PIDS].packet_count));
	seq_printf(m, "\n");
}
