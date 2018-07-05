/*
 * QEMU loongson 1a spi emulation
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
#include "hw/sysbus.h"
#include "hw/ssi/ssi.h"

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;
	uint8_t spcr;
	uint8_t spsr;
	uint8_t spdr;
	uint8_t sper;
	uint8_t param;
	uint8_t cs;
	uint8_t timing;


    /* The FIFO head points to the next empty entry.  */
    uint32_t tx_fifo_wptr;
    uint32_t tx_fifo_rptr;
    uint32_t rx_fifo_wptr;
    uint32_t rx_fifo_rptr;
    int tx_fifo_len;
    int rx_fifo_len;
    uint16_t tx_fifo[8];
    uint16_t rx_fifo[8];
    qemu_irq irq;
    qemu_irq cs_line[4];
    SSIBus *ssi;
} ls1a_spi_state;
#define next_ptr(p) ((p+1)&7)


#define 	W_FULL  	(1<<3)
#define 	W_EMPTY  	(1<<2)
#define 	R_FULL  	(1<<1)
#define 	R_EMPTY  	(1<<0)

static void ls1a_spi_update(ls1a_spi_state *s)
{
			s->spsr &= ~W_FULL;
			s->spsr |= W_EMPTY;
			if(s->rx_fifo_wptr!=s->rx_fifo_rptr)
			s->spsr &= ~R_EMPTY;
			else s->spsr |= R_EMPTY;

			if(next_ptr(s->rx_fifo_wptr)==s->rx_fifo_rptr)
			s->spsr |= R_FULL;
			else 
			s->spsr &= ~R_FULL;

			if(s->spsr&R_FULL) qemu_set_irq(s->irq,1);
			else qemu_set_irq(s->irq,0);
}



static uint64_t ls1a_spi_read(void *opaque, hwaddr offset, unsigned size)
{
    ls1a_spi_state *s = (ls1a_spi_state *)opaque;
    int val;

    switch (offset) {
    case 0x00: /* spcr */
      return s->spcr;
    case 0x01: /* spsr */
      return s->spsr;
    case 0x02: /* spdr */
		if(!(s->spsr&R_EMPTY))
		{
		val=s->rx_fifo[s->rx_fifo_rptr];
		s->rx_fifo_rptr=next_ptr(s->rx_fifo_rptr);
		}
		else val=0;
		ls1a_spi_update(s);
        return val;
    case 0x03: /* sper */
        return s->sper;
    case 0x04:
	return s->param;
    case 0x05: /*setcs*/
	return s->cs;
	break;
    case 0x06:
	return s->timing;
	break;
    default:
        return 0;
    }
}

static void ls1a_spi_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    ls1a_spi_state *s = (ls1a_spi_state *)opaque;
	uint32_t val;

    switch (offset) {
    case 0x00: /* spcr */
        s->spcr = value;
        break;
    case 0x01: /* spsr */
        s->spsr &= ~value;
        break;
    case 0x02: /* DR */
        val = ssi_transfer(s->ssi, value);
		s->spsr &= ~0x80;
		if(!(s->spsr&R_FULL))
		{
		s->rx_fifo[s->rx_fifo_wptr]=val; 
		s->rx_fifo_wptr=next_ptr(s->rx_fifo_wptr);
		s->spsr |= 0x80;
		}
			ls1a_spi_update(s);
        break;
    case 0x3: /* sper */
        s->sper = value;
        break;
    case 0x4:
	s->param = value;
	break;
    case 0x5: /*softcs*/;
	{
		int i;
		for(i=0;i<4;i++)
		{
			if((value&(0x11<<i))==(0x11<<i))
				qemu_irq_raise(s->cs_line[i]);
			else if((value&(0x11<<i))==(0x01<<i))
				qemu_irq_lower(s->cs_line[i]);
		}
	}
	s->cs = value;
	break;
    case 0x6:
	s->timing = value;
	break;
    default:
		;
    }
}

static void ls1a_spi_reset(ls1a_spi_state *s)
{
    s->rx_fifo_len = 0;
    s->tx_fifo_len = 0;
	s->spcr=0;
	s->spsr=5;
	s->sper=0;
}



static const MemoryRegionOps ls1a_spi_ops = {
    .read = ls1a_spi_read,
    .write = ls1a_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

#define TYPE_SYS_BUS_LS1ASPI "ls1a_spi"

#define SYS_BUS_LS1ASPI(obj) \
    OBJECT_CHECK(ls1a_spi_state, (obj), TYPE_SYS_BUS_LS1ASPI)

static int ls1a_spi_init(SysBusDevice *dev)
{
    ls1a_spi_state *d = SYS_BUS_LS1ASPI(dev);
    int i;

    memory_region_init_io(&d->iomem, NULL, &ls1a_spi_ops, (void *)d, "ls1a spi", 0x7);

    sysbus_init_irq(dev, &d->irq);
    for(i=0;i<4;i++)
    sysbus_init_irq(dev, &d->cs_line[i]);
    sysbus_init_mmio(dev, &d->iomem);
    d->ssi = ssi_create_bus(DEVICE(dev), "ssi");
    ls1a_spi_reset(d);

    return 0;
}


static void ls1a_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ls1a_spi_init;
    dc->desc = "ls1a i2c";
}

static const TypeInfo ls1a_spi_info = {
    .name          = "ls1a_spi",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ls1a_spi_state),
    .class_init    = ls1a_spi_class_init,
};


static void ls1a_spi_register_types(void)
{
    type_register_static(&ls1a_spi_info);
}

type_init(ls1a_spi_register_types)

