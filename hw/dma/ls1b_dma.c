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
#include "hw/hw.h"
#include "hw/sysbus.h"
#define DPRINTF(a...) //printf(a)

/*irq is placed on each module now */
typedef struct dma_sysbus_state {
	SysBusDevice busdev;
	MemoryRegion dma;
} dma_sysbus_state;


void ls1a_ac97_rec_set_dmaaddr(uint32_t val);
void ls1a_ac97_play_set_dmaaddr(uint32_t val);
void ls1a_nand_set_dmaaddr(uint32_t val);

static uint64_t dma_dma_readl(void *ptr, hwaddr addr, unsigned size)
{
	uint32_t val = 0;
	DPRINTF("dma_dma_readl:  (addr 0x%08X), val 0x%08X\n", (unsigned) addr, val);
	return val;
}

void __attribute__((weak)) ls1a_nand_set_dmaaddr(uint32_t val)
{
}


void __attribute__((weak)) ls1a_ac97_play_set_dmaaddr(uint32_t val)
{
}

void __attribute__((weak)) ls1a_ac97_rec_set_dmaaddr(uint32_t val)
{
}

static void dma_dma_writel(void *ptr, hwaddr addr, uint64_t val, unsigned size)
{
	int chn = val&3;
	switch(chn)
	{
	  case 0:
		ls1a_nand_set_dmaaddr(val);
		break;
	  case 1:
		ls1a_ac97_play_set_dmaaddr(val);
		break;
	  case 2:
		ls1a_ac97_rec_set_dmaaddr(val);
		break;
	  default:
		break;
	}

	DPRINTF("dma_dma_writel:  (addr 0x%08X), val 0x%08X\n", (unsigned) addr, val);
}

static const MemoryRegionOps dma_ops = {
    .read = dma_dma_readl,
    .write = dma_dma_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

#define TYPE_SYS_BUS_LS1BDMA "ls1b_dma"

#define SYS_BUS_LS1BDMA(obj) \
    OBJECT_CHECK(dma_sysbus_state, (obj), TYPE_SYS_BUS_LS1BDMA)

static int dma_sysbus_init(SysBusDevice *dev)
{
    dma_sysbus_state *d = SYS_BUS_LS1BDMA(dev);
    memory_region_init_io(&d->dma, NULL, &dma_ops, (void *)d, "dma", 0x4);

    sysbus_init_mmio(dev, &d->dma);

    return 0;
}

static void dma_sysbus_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = dma_sysbus_init;
    dc->desc = "ls1b dma";
}

static const TypeInfo dma_sysbus_info = {
    .name          = "ls1b_dma",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(dma_sysbus_state),
    .class_init    = dma_sysbus_class_init,
};


static void dma_sysbus_register_types(void)
{
    type_register_static(&dma_sysbus_info);
}

type_init(dma_sysbus_register_types)
