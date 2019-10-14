/*-
 *   BSD LICENSE
 *
 *   Copyright (c) Intel Corporation.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "spdk/stdinc.h"

#include "ioat_internal.h"

#include "spdk/env.h"
#include "spdk/util.h"

#include "spdk_internal/log.h"
#include "spdk_internal/memory.h"
#include <emmintrin.h>

struct ioat_driver {
	pthread_mutex_t			lock;
	TAILQ_HEAD(, spdk_ioat_chan)	attached_chans;
	int num;//新加变量
};


static struct ioat_driver g_ioat_driver = {
	.lock = PTHREAD_MUTEX_INITIALIZER,
	.attached_chans = TAILQ_HEAD_INITIALIZER(g_ioat_driver.attached_chans),
	.num = 0
};

//*******************************************************************************************************************************************************//
//*******************************************************************************************************************************************************//
//*******************************************************************************************************************************************************//
//陈振科自创代码

volatile int select_mark = 0;
struct spdk_ioat_chan* chan_array[128];
pthread_spinlock_t select_lock;

static uint32_t ioat_get_ring_index(struct spdk_ioat_chan *ioat, uint32_t index);

static void ioat_get_ring_entry(struct spdk_ioat_chan *ioat, uint32_t index,
		    struct ioat_descriptor **desc,
		    union spdk_ioat_hw_desc **hw_desc);

static inline uint32_t ioat_get_active(struct spdk_ioat_chan *ioat);

void spdk_ioat_detach(struct spdk_ioat_chan *ioat);

#define unlikely(x)  __builtin_expect(!!(x), 0)


static struct ioat_descriptor *
ioat_prep_enqueue(struct spdk_ioat_chan *ioat, uint64_t dst,
	       uint64_t src, uint32_t len, uint32_t orig_head)
{
	struct ioat_descriptor *desc;
	union spdk_ioat_hw_desc *hw_desc;

	assert(len <= ioat->max_xfer_size);


	ioat_get_ring_entry(ioat, orig_head, &desc, &hw_desc);

	hw_desc->dma.u.control_raw = 0;
	hw_desc->dma.u.control.op = SPDK_IOAT_OP_COPY;

	hw_desc->dma.size = len;
	hw_desc->dma.src_addr = src;
	hw_desc->dma.dest_addr = dst;

	desc->callback_fn = NULL;
	desc->callback_arg = NULL;
	return desc;
}
static inline int
ioat_atomic32_cmpset(volatile uint32_t *dst, uint32_t exp, uint32_t src)
{
	uint8_t res;

	asm volatile(
			"lock ; "
			"cmpxchgl %[src], %[dst];"//隐含使用EAX寄存器，src与EAX是否相等，如果是dst等于EAX，否则保留原值
			"sete %[res];"
			: [res] "=a" (res),     /* output */
			  [dst] "=m" (*dst)
			: [src] "r" (src),      /* input */
			  "a" (exp),      /*a表示使用EAX寄存器 */
			  "m" (*dst)
			: "memory");            /* no-clobber list */
	return res;
}

void 
ioat_ring_move_head(struct spdk_ioat_chan *ioat,
		unsigned int n, uint32_t *old_head, uint32_t *new_head)
{
	int success;
	do {
		// printf("ioat_ring_move_head\n");
		*old_head = ioat->head;
		// if(ioat_get_active(ioat) <= n)continue;

		do {		
		spdk_compiler_barrier();
		} while(0);
		
		*new_head = *old_head + n;
		// printf("old_head = %u, new_head = %u\n",*old_head,*new_head);
		success = ioat_atomic32_cmpset(&ioat->head ,*old_head ,*new_head);	
		// printf("success = %d\n",success);
	}while (unlikely(success == 0));
	// printf("ioat->head = %u\n",ioat->head);
	
	
}

int 
ioat_ring_do_enqueue(struct spdk_ioat_chan *ioat, void *dst, void *src, uint64_t length, uint32_t prod_head, void *cb_arg, spdk_ioat_req_cb cb_fn)
{
	struct ioat_descriptor	*last_desc;
	uint64_t	remaining = length, op_size;
	uint64_t	vdst, vsrc;
	uint64_t	vdst_page, vsrc_page;
	uint64_t	pdst_page, psrc_page;
	uint32_t	orig_head = prod_head;


	if (!ioat) {
		return -EINVAL;
	}

	vdst = (uint64_t)dst;
	vsrc = (uint64_t)src;
	vdst_page = vsrc_page = 0;
	pdst_page = psrc_page = SPDK_VTOPHYS_ERROR;

	
	while (remaining) {
		if (_2MB_PAGE(vsrc) != vsrc_page) {			
			vsrc_page = _2MB_PAGE(vsrc);
			psrc_page = spdk_vtophys((void *)vsrc_page, NULL);
		}	
		if (_2MB_PAGE(vdst) != vdst_page) {
			vdst_page = _2MB_PAGE(vdst);
			pdst_page = spdk_vtophys((void *)vdst_page, NULL);
		}
		
		op_size = remaining;
		op_size = spdk_min(op_size, (VALUE_2MB - _2MB_OFFSET(vsrc)));
		op_size = spdk_min(op_size, (VALUE_2MB - _2MB_OFFSET(vdst)));
		op_size = spdk_min(op_size, ioat->max_xfer_size);
		remaining -= op_size;
		// printf("vsrc_page=%lx,",_2MB_PAGE(vsrc_page));
		// printf("psrc_page=%lx,offset=%8lx  ",_2MB_PAGE(psrc_page),_2MB_OFFSET(vsrc));
		// printf("vdst_page=%lx,",_2MB_PAGE(vdst_page));
		// printf("pdst_page=%lx,offset=%8lx  ",_2MB_PAGE(pdst_page),_2MB_OFFSET(vdst));
		// printf("op_size = %lx,remaining = %lx\n",op_size,remaining);
		last_desc = ioat_prep_enqueue(ioat,
					   pdst_page + _2MB_OFFSET(vdst),
					   psrc_page + _2MB_OFFSET(vsrc),
					   op_size,
					   orig_head);

		if (remaining == 0 ) {
			break;
		}

		vsrc += op_size;
		vdst += op_size;
		orig_head++;
	}
	
	if (last_desc) {
		last_desc->callback_fn = cb_fn;
		last_desc->callback_arg = cb_arg;
		// printf("callback is set 0x%lx,arg is 0x%lx\n",cb_fn,cb_arg);
	} 
	return 0;
}
#define max(a,b) a>b?a:b
struct spdk_ioat_chan**
ioat_select_chan(int num)
{
	int temp;
	struct spdk_ioat_chan **select = (struct spdk_ioat_chan *)malloc(num * sizeof(struct spdk_ioat_chan*));
	struct spdk_ioat_chan *chan;
	pthread_spin_lock(&select_lock);
	int max_temperature = 0;
	TAILQ_FOREACH(chan, &g_ioat_driver.attached_chans , tailq)
	{
		max_temperature = max(max_temperature, chan->temperature);
	}
	printf("max_temperature = %d\n", max_temperature);
	max_temperature *= 0.3;
	int i = 0;
	// printf("here!\n");
	TAILQ_FOREACH(chan, &g_ioat_driver.attached_chans , tailq)
	{
		if( chan->temperature <= max_temperature )
		{
			select[i++] = chan;
			chan->temperature++;
			printf("select channel 0x%x!\n",chan);
			if(i == num)break;
		}
	}
	printf("\n");
	pthread_spin_unlock(&select_lock);
	return select;
}

void 
ioat_cb(void *arg)
{
	int *p = (int*)arg;
	(*p)++;
	// printf("ioat_cb！ %d\n",*p);
	// printf("callback!\n");
}
void
ioat_ring_flush(struct spdk_ioat_chan *chan, uint32_t prod_head, uint32_t prod_next ) 
{
	uint32_t index = ioat_get_ring_index(chan, prod_next - 1);//错误出现在这里
	union spdk_ioat_hw_desc *hw_desc;
	hw_desc = &chan->hw_ring[index];
	volatile uint16_t *dmacount = &chan->regs->dmacount;

	spdk_compiler_barrier();	
	while (unlikely(*dmacount != (uint16_t)prod_head))
		_mm_pause();
	hw_desc->dma.u.control.completion_update = 1;
	*dmacount = (uint16_t)prod_next;
	chan->temperature--;
	// printf("chan->head = %" PRIu32 " core %d \n",chan->head,spdk_env_get_current_core());
	// printf("chan->regs->dmacount = %" PRIu16 "\n",*dmacount);
	// printf("chan->comp_update = %" PRIu64 "\n",*chan->comp_update);

}


unsigned int
ioat_ring_enqueue(struct spdk_ioat_chan *ioat ,void *dst, void *src, uint64_t length, int *temp)
{
	uint32_t prod_head, prod_next;
	unsigned int n = length / ioat->max_xfer_size;
	if(_2MB_OFFSET(src) || _2MB_OFFSET(dst))
	{
		printf("src or dst is not align!\n");
		return 0;
	}
	// else printf("_2MB_OFFSET(src) = %lu\n_2MB_OFFSET(dst) = %lu\n",_2MB_OFFSET(src),_2MB_OFFSET(dst));
	//在这里需要考虑对齐
	// printf("ioat_ring_move_head,core %d\n",spdk_env_get_current_core());

	ioat_ring_move_head(ioat, n, &prod_head, &prod_next);
	ioat_ring_do_enqueue(ioat, dst, src, length, prod_head, temp, ioat_cb);
	ioat_ring_flush(ioat, prod_head, prod_next);

	
	return 0;
}

int ioat_length_to_num(uint64_t length)
{
	int num_chan = length / pow(2, 30);
	
	if(length & MASK_2MB)num_chan++;
	// printf("num_chan = %d\n", num_chan);
	return num_chan;
}

int
ioat_copy(void *dst, void *src, uint64_t length, int *flag)
{
	struct spdk_ioat_chan *chan;
	uint8_t *_src = (uint8_t*)src;
	uint8_t *_dst = (uint8_t*)dst;
	
	// int num_chan = ioat_length_to_num(length);
	struct spdk_ioat_chan **select = ioat_select_chan(num_chan);
	int size = length / num_chan;
	
	// uint64_t size = length / g_ioat_driver.num;
	// uint64_t size = num_per_2MB * VALUE_2MB;
	// printf("size = %" PRIu64 "\n", num_per_2MB);
	*flag = 0;
	TAILQ_FOREACH(chan, &g_ioat_driver.attached_chans , tailq)
	for(int i = 0; i < num_chan; i++)
	{
		
		ioat_ring_enqueue(
			select[i],
			_dst,
			_src,
			size,
			flag);
		_dst+=size;
		_src+=size;
		
	}
	return num_chan;
}

int ioat_poll(void)
{
	if(TAILQ_EMPTY(&g_ioat_driver.attached_chans))
	{
		printf("work_poll: worker_chan is empty!\n");
		return 0;
	}
	struct spdk_ioat_chan *chan;

	// printf("polling\n");
	TAILQ_FOREACH(chan , &g_ioat_driver.attached_chans ,tailq)
	{
		spdk_ioat_process_events(chan);
	}
	return 0;
}

void ioat_cleanup(void)
{
	struct spdk_ioat_chan *chan;
	TAILQ_FOREACH(chan, &g_ioat_driver.attached_chans , tailq)
	{
		spdk_ioat_detach(chan);
	}
}

//*******************************************************************************************************************************************************//
//*******************************************************************************************************************************************************//
//*******************************************************************************************************************************************************//


static uint64_t
ioat_get_chansts(struct spdk_ioat_chan *ioat)
{
	return spdk_mmio_read_8(&ioat->regs->chansts);
}

static void
ioat_write_chancmp(struct spdk_ioat_chan *ioat, uint64_t addr)
{
	spdk_mmio_write_8(&ioat->regs->chancmp, addr);
}

static void
ioat_write_chainaddr(struct spdk_ioat_chan *ioat, uint64_t addr)
{
	spdk_mmio_write_8(&ioat->regs->chainaddr, addr);
}

static inline void
ioat_suspend(struct spdk_ioat_chan *ioat)
{
	ioat->regs->chancmd = SPDK_IOAT_CHANCMD_SUSPEND;
}

static inline void
ioat_reset(struct spdk_ioat_chan *ioat)
{
	ioat->regs->chancmd = SPDK_IOAT_CHANCMD_RESET;
}

static inline uint32_t
ioat_reset_pending(struct spdk_ioat_chan *ioat)
{
	uint8_t cmd;

	cmd = ioat->regs->chancmd;
	return (cmd & SPDK_IOAT_CHANCMD_RESET) == SPDK_IOAT_CHANCMD_RESET;
}

static int
ioat_map_pci_bar(struct spdk_ioat_chan *ioat)
{
	int regs_bar, rc;
	void *addr;
	uint64_t phys_addr, size;

	regs_bar = 0;
	rc = spdk_pci_device_map_bar(ioat->device, regs_bar, &addr, &phys_addr, &size);//将dev->mem_resource[bar]的addr、phys_addr、len分别赋值
	if (rc != 0 || addr == NULL) {
		SPDK_ERRLOG("pci_device_map_range failed with error code %d\n",
			    rc);
		return -1;
	}

	ioat->regs = (volatile struct spdk_ioat_registers *)addr;

	return 0;
}

static int
ioat_unmap_pci_bar(struct spdk_ioat_chan *ioat)
{
	int rc = 0;
	void *addr = (void *)ioat->regs;

	if (addr) {
		rc = spdk_pci_device_unmap_bar(ioat->device, 0, addr);
	}
	return rc;
}


static inline uint32_t
ioat_get_active(struct spdk_ioat_chan *ioat)
{
	return (ioat->head - ioat->tail) & ((1 << ioat->ring_size_order) - 1);
}

static inline uint32_t
ioat_get_ring_space(struct spdk_ioat_chan *ioat)
{
	return (1 << ioat->ring_size_order) - ioat_get_active(ioat) - 1;
}

static uint32_t
ioat_get_ring_index(struct spdk_ioat_chan *ioat, uint32_t index)
{
	return index & ((1 << ioat->ring_size_order) - 1);
}

static void
ioat_get_ring_entry(struct spdk_ioat_chan *ioat, uint32_t index,
		    struct ioat_descriptor **desc,
		    union spdk_ioat_hw_desc **hw_desc)
{
	uint32_t i = ioat_get_ring_index(ioat, index);

	*desc = &ioat->ring[i];
	*hw_desc = &ioat->hw_ring[i];
}

static void
ioat_submit_single(struct spdk_ioat_chan *ioat)
{
	ioat->head++;
	// printf("ioat->head=%u\n",ioat->head);
}

void
spdk_ioat_flush(struct spdk_ioat_chan *ioat)
{
	uint32_t index = ioat_get_ring_index(ioat, ioat->head - 1);
	union spdk_ioat_hw_desc *hw_desc;

	hw_desc = &ioat->hw_ring[index];
	hw_desc->dma.u.control.completion_update = 1;
	ioat->regs->dmacount = (uint16_t)ioat->head;
}

static struct ioat_descriptor *
ioat_prep_null(struct spdk_ioat_chan *ioat)
{
	struct ioat_descriptor *desc;
	union spdk_ioat_hw_desc *hw_desc;

	if (ioat_get_ring_space(ioat) < 1) {
		return NULL;
	}

	ioat_get_ring_entry(ioat, ioat->head, &desc, &hw_desc);

	hw_desc->dma.u.control_raw = 0;
	hw_desc->dma.u.control.op = SPDK_IOAT_OP_COPY;
	hw_desc->dma.u.control.null = 1;

	hw_desc->dma.size = 8;
	hw_desc->dma.src_addr = 0;
	hw_desc->dma.dest_addr = 0;

	desc->callback_fn = NULL;
	desc->callback_arg = NULL;

	ioat_submit_single(ioat);

	return desc;
}

static struct ioat_descriptor *
ioat_prep_copy(struct spdk_ioat_chan *ioat, uint64_t dst,
	       uint64_t src, uint32_t len)
{
	struct ioat_descriptor *desc;
	union spdk_ioat_hw_desc *hw_desc;

	assert(len <= ioat->max_xfer_size);

	if (ioat_get_ring_space(ioat) < 1) {
		return NULL;
	}

	ioat_get_ring_entry(ioat, ioat->head, &desc, &hw_desc);
	// printf("ioat->head=%d \n", ioat->head);

	hw_desc->dma.u.control_raw = 0;
	hw_desc->dma.u.control.op = SPDK_IOAT_OP_COPY;

	hw_desc->dma.size = len;
	hw_desc->dma.src_addr = src;
	hw_desc->dma.dest_addr = dst;

	desc->callback_fn = NULL;
	desc->callback_arg = NULL;

	ioat_submit_single(ioat);

	return desc;
}

static struct ioat_descriptor *
ioat_prep_fill(struct spdk_ioat_chan *ioat, uint64_t dst,
	       uint64_t fill_pattern, uint32_t len)
{
	struct ioat_descriptor *desc;
	union spdk_ioat_hw_desc *hw_desc;

	assert(len <= ioat->max_xfer_size);

	if (ioat_get_ring_space(ioat) < 1) {
		return NULL;
	}

	ioat_get_ring_entry(ioat, ioat->head, &desc, &hw_desc);

	hw_desc->fill.u.control_raw = 0;
	hw_desc->fill.u.control.op = SPDK_IOAT_OP_FILL;

	hw_desc->fill.size = len;
	hw_desc->fill.src_data = fill_pattern;
	hw_desc->fill.dest_addr = dst;

	desc->callback_fn = NULL;
	desc->callback_arg = NULL;

	ioat_submit_single(ioat);

	return desc;
}

static int ioat_reset_hw(struct spdk_ioat_chan *ioat)
{
	int timeout;
	uint64_t status;
	uint32_t chanerr;
	int rc;

	status = ioat_get_chansts(ioat);
	if (is_ioat_active(status) || is_ioat_idle(status)) {
		ioat_suspend(ioat);
	}

	timeout = 20; /* in milliseconds */
	while (is_ioat_active(status) || is_ioat_idle(status)) {
		spdk_delay_us(1000);
		timeout--;
		if (timeout == 0) {
			SPDK_ERRLOG("timed out waiting for suspend\n");
			return -1;
		}
		status = ioat_get_chansts(ioat);
	}

	/*
	 * Clear any outstanding errors.
	 * CHANERR is write-1-to-clear, so write the current CHANERR bits back to reset everything.
	 */
	chanerr = ioat->regs->chanerr;
	ioat->regs->chanerr = chanerr;

	if (ioat->regs->cbver < SPDK_IOAT_VER_3_3) {
		rc = spdk_pci_device_cfg_read32(ioat->device, &chanerr,
						SPDK_IOAT_PCI_CHANERR_INT_OFFSET);
		if (rc) {
			SPDK_ERRLOG("failed to read the internal channel error register\n");
			return -1;
		}

		spdk_pci_device_cfg_write32(ioat->device, chanerr,
					    SPDK_IOAT_PCI_CHANERR_INT_OFFSET);
	}

	ioat_reset(ioat);

	timeout = 20;
	while (ioat_reset_pending(ioat)) {
		spdk_delay_us(1000);
		timeout--;
		if (timeout == 0) {
			SPDK_ERRLOG("timed out waiting for reset\n");
			return -1;
		}
	}

	return 0;
}



static int
ioat_process_channel_events(struct spdk_ioat_chan *ioat)
{
	struct ioat_descriptor *desc;
	uint64_t status, completed_descriptor, hw_desc_phys_addr;
	uint32_t tail;

	if (ioat->head == ioat->tail) {
	// printf("ioat->head=%u  ",ioat->head);
	// printf("ioat->tail=%u\n",ioat->tail);
	// printf("ioat->head == ioat->tail\n");
	return 0;
	}

	status = *ioat->comp_update;
	completed_descriptor = status & SPDK_IOAT_CHANSTS_COMPLETED_DESCRIPTOR_MASK;

	if (is_ioat_halted(status)) {
		SPDK_ERRLOG("Channel halted (%x)\n", ioat->regs->chanerr);
		printf("Channel halted\n");
		 return -1; 
	}
	// printf("*ioat->comp_update = %" PRIu16 "\n",*ioat->comp_update);
	if (completed_descriptor == ioat->last_seen) {
		
		// printf("completed_descriptor == ioat->last_seen \n");
		return 0;
	} 
	do {
		tail = ioat_get_ring_index(ioat, ioat->tail);
		desc = &ioat->ring[tail];
		// printf("tail=%4u ",tail);
		if (desc->callback_fn) {
			// printf("callback 0x%x has been called\n",desc->callback_fn);
			desc->callback_fn(desc->callback_arg);
			
		}
//		printf("hw_desc_phys_addr=%lx,completed_descriptor=%lx\n",hw_desc_phys_addr,completed_descriptor);
		hw_desc_phys_addr = desc->phys_addr;
		ioat->tail++;
	} while (hw_desc_phys_addr != completed_descriptor);

	ioat->last_seen = hw_desc_phys_addr;
	return 0;
}

static void
ioat_channel_destruct(struct spdk_ioat_chan *ioat)
{
	ioat_unmap_pci_bar(ioat);

	if (ioat->ring) {
		free(ioat->ring);
	}

	if (ioat->hw_ring) {
		spdk_dma_free(ioat->hw_ring);
	}

	if (ioat->comp_update) {
		spdk_dma_free((void *)ioat->comp_update);
		ioat->comp_update = NULL;
	}
}

static int
ioat_channel_start(struct spdk_ioat_chan *ioat) //初始化DMA通道
{
	uint8_t xfercap, version;
	uint64_t status;
	int i, num_descriptors;
	uint64_t comp_update_bus_addr = 0;
	uint64_t phys_addr;

	if (ioat_map_pci_bar(ioat) != 0) {//为ioat->regs赋值
		SPDK_ERRLOG("ioat_map_pci_bar() failed\n");
		return -1;
	}

	version = ioat->regs->cbver;
	if (version < SPDK_IOAT_VER_3_0) {
		SPDK_ERRLOG(" unsupported IOAT version %u.%u\n",
			    version >> 4, version & 0xF);
		return -1;
	}

	/* Always support DMA copy */
	ioat->dma_capabilities = SPDK_IOAT_ENGINE_COPY_SUPPORTED;
	if (ioat->regs->dmacapability & SPDK_IOAT_DMACAP_BFILL) {
		ioat->dma_capabilities |= SPDK_IOAT_ENGINE_FILL_SUPPORTED;
	}
	xfercap = ioat->regs->xfercap;

	/* Only bits [4:0] are valid. */
	xfercap &= 0x1f;
	if (xfercap == 0) {
		/* 0 means 4 GB max transfer size. */
		ioat->max_xfer_size = 1ULL << 32;
	} else if (xfercap < 12) {
		/* XFERCAP must be at least 12 (4 KB) according to the spec. */
		SPDK_ERRLOG("invalid XFERCAP value %u\n", xfercap);
		return -1;
	} else {
		ioat->max_xfer_size = 1U << xfercap;
	}

	ioat->comp_update = spdk_dma_zmalloc(sizeof(*ioat->comp_update), SPDK_IOAT_CHANCMP_ALIGN,
					     &comp_update_bus_addr);
	if (ioat->comp_update == NULL) {
		return -1;
	}

	ioat->ring_size_order = IOAT_DEFAULT_ORDER;

	num_descriptors = 1 << ioat->ring_size_order;

	ioat->ring = calloc(num_descriptors, sizeof(struct ioat_descriptor));
	if (!ioat->ring) {
		return -1;
	}

	ioat->hw_ring = spdk_dma_zmalloc(num_descriptors * sizeof(union spdk_ioat_hw_desc), 64,
					 NULL);
	if (!ioat->hw_ring) {
		return -1;
	}

	for (i = 0; i < num_descriptors; i++) {
		phys_addr = spdk_vtophys(&ioat->hw_ring[i], NULL);
		if (phys_addr == SPDK_VTOPHYS_ERROR) {
			SPDK_ERRLOG("Failed to translate descriptor %u to physical address\n", i);
			return -1;
		}

		ioat->ring[i].phys_addr = phys_addr;
		ioat->hw_ring[ioat_get_ring_index(ioat, i - 1)].generic.next = phys_addr;
	}

	ioat->head = 0;
	ioat->tail = 0;
	ioat->temperature = 0;//自行添加
	ioat->last_seen = 0;

	ioat_reset_hw(ioat);

	ioat->regs->chanctrl = SPDK_IOAT_CHANCTRL_ANY_ERR_ABORT_EN;
	ioat_write_chancmp(ioat, comp_update_bus_addr);
	ioat_write_chainaddr(ioat, ioat->ring[0].phys_addr);

	ioat_prep_null(ioat);
	spdk_ioat_flush(ioat);

	i = 100;
	while (i-- > 0) {
		spdk_delay_us(100);
		status = ioat_get_chansts(ioat);
		if (is_ioat_idle(status)) {
			break;
		}
	}

	if (is_ioat_idle(status)) {
		ioat_process_channel_events(ioat);
	} else {
		SPDK_ERRLOG("could not start channel: status = %p\n error = %#x\n",
			    (void *)status, ioat->regs->chanerr);
		return -1;
	}

	return 0;
}

/* Caller must hold g_ioat_driver.lock */
static struct spdk_ioat_chan *
ioat_attach(struct spdk_pci_device *device)
{
	struct spdk_ioat_chan *ioat;
	uint32_t cmd_reg;

	ioat = calloc(1, sizeof(struct spdk_ioat_chan));
	if (ioat == NULL) {
		return NULL;
	}

	/* Enable PCI busmaster. */
	spdk_pci_device_cfg_read32(device, &cmd_reg, 4);
	cmd_reg |= 0x4;
	spdk_pci_device_cfg_write32(device, cmd_reg, 4);

	ioat->device = device;

	if (ioat_channel_start(ioat) != 0) {
		ioat_channel_destruct(ioat);
		free(ioat);
		return NULL;
	}

	return ioat;
}

struct ioat_enum_ctx {
	spdk_ioat_probe_cb probe_cb;
	spdk_ioat_attach_cb attach_cb;
	void *cb_ctx;
};

/* This function must only be called while holding g_ioat_driver.lock */
static int
ioat_enum_cb(void *ctx, struct spdk_pci_device *pci_dev)
{
	struct ioat_enum_ctx *enum_ctx = ctx;
	struct spdk_ioat_chan *ioat;

	/* Verify that this device is not already attached */
	TAILQ_FOREACH(ioat, &g_ioat_driver.attached_chans, tailq) {
		/*
		 * NOTE: This assumes that the PCI abstraction layer will use the same device handle
		 *  across enumerations; we could compare by BDF instead if this is not true.
		 */
		if (pci_dev == ioat->device) {
			return 0;
		}
	}

	if (enum_ctx->probe_cb(enum_ctx->cb_ctx, pci_dev)) {
		/*
		 * Since I/OAT init is relatively quick, just perform the full init during probing.
		 *  If this turns out to be a bottleneck later, this can be changed to work like
		 *  NVMe with a list of devices to initialize in parallel.
		 */
		ioat = ioat_attach(pci_dev);
		if (ioat == NULL) {
			SPDK_ERRLOG("ioat_attach() failed\n");
			return -1;
		}

		TAILQ_INSERT_TAIL(&g_ioat_driver.attached_chans, ioat, tailq);
		ioat->id = g_ioat_driver.num;//自创代码
		g_ioat_driver.num++;//自创代码
		enum_ctx->attach_cb(enum_ctx->cb_ctx, pci_dev, ioat);
	}

	return 0;
}

int
spdk_ioat_probe(void *cb_ctx, spdk_ioat_probe_cb probe_cb, spdk_ioat_attach_cb attach_cb)
{
	int rc;
	struct ioat_enum_ctx enum_ctx;

	pthread_mutex_lock(&g_ioat_driver.lock);

	enum_ctx.probe_cb = probe_cb;
	enum_ctx.attach_cb = attach_cb;
	enum_ctx.cb_ctx = cb_ctx;

	rc = spdk_pci_enumerate(spdk_pci_ioat_get_driver(), ioat_enum_cb, &enum_ctx);

	pthread_mutex_unlock(&g_ioat_driver.lock);
    pthread_spin_init(&select_lock, PTHREAD_PROCESS_PRIVATE);//陈振科添加
	return rc;
}

void
spdk_ioat_detach(struct spdk_ioat_chan *ioat)
{
	struct ioat_driver	*driver = &g_ioat_driver;

	/* ioat should be in the free list (not registered to a thread)
	 * when calling ioat_detach().
	 */
	pthread_mutex_lock(&driver->lock);
	TAILQ_REMOVE(&driver->attached_chans, ioat, tailq);
	pthread_mutex_unlock(&driver->lock);

	ioat_channel_destruct(ioat);
	free(ioat);
}

int
spdk_ioat_build_copy(struct spdk_ioat_chan *ioat, void *cb_arg, spdk_ioat_req_cb cb_fn,
		     void *dst, const void *src, uint64_t nbytes)
{
	struct ioat_descriptor	*last_desc;
	uint64_t	remaining, op_size;
	uint64_t	vdst, vsrc;
	uint64_t	vdst_page, vsrc_page;
	uint64_t	pdst_page, psrc_page;
	uint32_t	orig_head;

	if (!ioat) {
		return -EINVAL;
	}

	orig_head = ioat->head;

	vdst = (uint64_t)dst;
	vsrc = (uint64_t)src;
	vdst_page = vsrc_page = 0;
	pdst_page = psrc_page = SPDK_VTOPHYS_ERROR;

	remaining = nbytes;
	while (remaining) {
		if (_2MB_PAGE(vsrc) != vsrc_page) {			
			vsrc_page = _2MB_PAGE(vsrc);
			psrc_page = spdk_vtophys((void *)vsrc_page, NULL);
		}	
		if (_2MB_PAGE(vdst) != vdst_page) {
			vdst_page = _2MB_PAGE(vdst);
			pdst_page = spdk_vtophys((void *)vdst_page, NULL);
		}
		
		op_size = remaining;
		op_size = spdk_min(op_size, (VALUE_2MB - _2MB_OFFSET(vsrc)));
		op_size = spdk_min(op_size, (VALUE_2MB - _2MB_OFFSET(vdst)));
		op_size = spdk_min(op_size, ioat->max_xfer_size);
		remaining -= op_size;
		// printf("vsrc_page=%lx,",_2MB_PAGE(vsrc_page));
		// printf("psrc_page=%lx,offset=%8lx  ",_2MB_PAGE(psrc_page),_2MB_OFFSET(vsrc));
		// printf("vdst_page=%lx,",_2MB_PAGE(vdst_page));
		// printf("pdst_page=%lx,offset=%8lx  ",_2MB_PAGE(pdst_page),_2MB_OFFSET(vdst));
		// printf("op_size=%lx,remaining=%lx\n",op_size,remaining);
		last_desc = ioat_prep_copy(ioat,
					   pdst_page + _2MB_OFFSET(vdst),
					   psrc_page + _2MB_OFFSET(vsrc),
					   op_size);

		if (remaining == 0 || last_desc == NULL) {
			break;
		}

		vsrc += op_size;
		vdst += op_size;

	}
	/* Issue null descriptor for null transfer */
	if (nbytes == 0) {
		last_desc = ioat_prep_null(ioat);
	}

	if (last_desc) {
		last_desc->callback_fn = cb_fn;
		last_desc->callback_arg = cb_arg;
		// printf("callback is set 0x%lx,arg is 0x%lx\n",cb_fn,cb_arg);
	} else {
		/*
		 * Ran out of descriptors in the ring - reset head to leave things as they were
		 * in case we managed to fill out any descriptors.
		 */
		ioat->head = orig_head;
		return -ENOMEM;
	}

	return 0;
}

int
spdk_ioat_submit_copy(struct spdk_ioat_chan *ioat, void *cb_arg, spdk_ioat_req_cb cb_fn,
		      void *dst, const void *src, uint64_t nbytes)
{
	int rc;

	rc = spdk_ioat_build_copy(ioat, cb_arg, cb_fn, dst, src, nbytes);
	if (rc != 0) {
		return rc;
	}
	printf("chan 0x%p,head= %" PRIu32 ",tail= %" PRIu32 "\n", ioat, ioat->head,ioat->tail);
	spdk_ioat_flush(ioat);
	return 0;
}

int
spdk_ioat_build_fill(struct spdk_ioat_chan *ioat, void *cb_arg, spdk_ioat_req_cb cb_fn,
		     void *dst, uint64_t fill_pattern, uint64_t nbytes)
{
	struct ioat_descriptor	*last_desc = NULL;
	uint64_t	remaining, op_size;
	uint64_t	vdst;
	uint32_t	orig_head;

	if (!ioat) {
		return -EINVAL;
	}

	if (!(ioat->dma_capabilities & SPDK_IOAT_ENGINE_FILL_SUPPORTED)) {
		SPDK_ERRLOG("Channel does not support memory fill\n");
		return -1;
	}

	orig_head = ioat->head;

	vdst = (uint64_t)dst;
	remaining = nbytes;
	
	while (remaining) {
		op_size = remaining;
		op_size = spdk_min(op_size, (VALUE_2MB - _2MB_OFFSET(vdst)));
		op_size = spdk_min(op_size, ioat->max_xfer_size);
		remaining -= op_size;

		last_desc = ioat_prep_fill(ioat,
					   spdk_vtophys((void *)vdst, NULL),
					   fill_pattern,
					   op_size);

		if (remaining == 0 || last_desc == NULL) {
			break;
		}

		vdst += op_size;
	}

	if (last_desc) {
		last_desc->callback_fn = cb_fn;
		last_desc->callback_arg = cb_arg;
	} else {
		/*
		 * Ran out of descriptors in the ring - reset head to leave things as they were
		 * in case we managed to fill out any descriptors.
		 */
		ioat->head = orig_head;
		return -ENOMEM;
	}

	return 0;
}

int
spdk_ioat_submit_fill(struct spdk_ioat_chan *ioat, void *cb_arg, spdk_ioat_req_cb cb_fn,
		      void *dst, uint64_t fill_pattern, uint64_t nbytes)
{
	int rc;

	rc = spdk_ioat_build_fill(ioat, cb_arg, cb_fn, dst, fill_pattern, nbytes);
	if (rc != 0) {
		return rc;
	}

	spdk_ioat_flush(ioat);
	return 0;
}

uint32_t
spdk_ioat_get_dma_capabilities(struct spdk_ioat_chan *ioat)
{
	if (!ioat) {
		return 0;
	}
	return ioat->dma_capabilities;
}

int
spdk_ioat_process_events(struct spdk_ioat_chan *ioat)
{
	return ioat_process_channel_events(ioat);
}

SPDK_LOG_REGISTER_COMPONENT("ioat", SPDK_LOG_IOAT)
