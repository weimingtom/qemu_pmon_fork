/*
 * QEMU loongson 1b dma emulation
 *
 * Copyright (c) 2013 qiaochong@loongson.cn
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#define DPRINTF(a...) printf(a)

typedef struct dma_fifo_regs{
int source;
int dest;
int rcount;
int wcount;
int command;
} dma_fifo_regs_t;

enum udma_type
{
DMA_START = 1,
WDMA_DONE = 2,
SOFT_RESET = 4,
MODE_DMA128 = 0,
MODE_DMA32 = 0x8,
MODE_DMA8 = 0x10,
RDMA_START = 0x20,
RDMA_DONE = 0x40,
INTMODE_WONLY = 0x80,
INTMODE_RW = 0x0
};

/*irq is placed on each module now */
typedef struct dma_sysbus_state {
	SysBusDevice busdev;
	MemoryRegion dma;
	qemu_irq irq;
        dma_fifo_regs_t udma[4];
} dma_sysbus_state;


void ls1a_ac97_rec_set_dmaaddr(uint32_t val);
void ls1a_ac97_play_set_dmaaddr(uint32_t val);
void ls1b_nand_set_dmaaddr(uint32_t val);

static uint64_t dma_dma_readl(void *ptr, hwaddr addr, unsigned size)
{
	uint64_t val;
	int ch, offs;
	dma_fifo_regs_t *udma;
	dma_sysbus_state *d = ptr;
	ch = addr/32;
	offs = addr & 0x1f;
	udma = &d->udma[ch];
        val = *(int *)((void *)udma+offs);
	DPRINTF("dma_dma_readl:  (addr 0x%08X), val 0x%08X\n", (unsigned) addr, (unsigned)val);
	return val;
}

static void dma_dma_writel(void *ptr, hwaddr addr, uint64_t val, unsigned size)
{
int ch, offs;
dma_fifo_regs_t *udma;
dma_sysbus_state *d = ptr;
ch = addr/32;
offs = addr & 0x1f;
udma = &d->udma[ch];

	switch(offs)
	{
		case 0x0:
		  udma->source = val;
		  break;
		case 0x4:
		  udma->dest = val;
		  break;
		case 0x8:
		  udma->rcount = val;
		  break;
		case 0xc:
		  udma->wcount = val;
		  break;
		case 0x10:
		  udma->command =((udma->command & ~val & (RDMA_DONE|WDMA_DONE))) | (val & ~(RDMA_DONE|WDMA_DONE));
		  if(val & DMA_START)
		  {
			  char *buf;
			  int i;
			  buf = malloc(udma->rcount);
			  cpu_physical_memory_read(udma->source,buf,udma->rcount);
			  for(i=0;i<udma->rcount;i++)
				  buf[i] ^= 0x55;
			  cpu_physical_memory_write(udma->dest,buf,udma->wcount);
			  free(buf);
			  udma->command |= RDMA_DONE|WDMA_DONE;

			  qemu_irq_raise(d->irq);
		  }
		  else if((udma->command & (RDMA_DONE|WDMA_DONE)) == 0)
			  qemu_irq_lower(d->irq);
                  
		break;
	  default:
		break;
	}

	DPRINTF("dma_dma_writel:  (addr 0x%08X), val 0x%08X\n", (unsigned) addr, (unsigned)val);
}

static const MemoryRegionOps dma_ops = {
    .read = dma_dma_readl,
    .write = dma_dma_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

#define TYPE_SYS_BUS_LS1BDMA "ls1gpa_dma"

#define SYS_BUS_LS1BDMA(obj) \
    OBJECT_CHECK(dma_sysbus_state, (obj), TYPE_SYS_BUS_LS1BDMA)

static int dma_sysbus_init(SysBusDevice *dev)
{
    dma_sysbus_state *d = SYS_BUS_LS1BDMA(dev);
    memory_region_init_io(&d->dma, NULL, &dma_ops, (void *)d, "dma", 0x80);

    sysbus_init_mmio(dev, &d->dma);
    sysbus_init_irq(dev, &d->irq);

    return 0;
}

static void dma_sysbus_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = dma_sysbus_init;
    dc->desc = "ls1gpa dma";
}

static const TypeInfo dma_sysbus_info = {
    .name          = "ls1gpa_dma",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(dma_sysbus_state),
    .class_init    = dma_sysbus_class_init,
};


static void dma_sysbus_register_types(void)
{
    type_register_static(&dma_sysbus_info);
}

type_init(dma_sysbus_register_types)
