/*
 * CAN bus emulation for ls1a. 
 * 
 * Author: Zeng Lu <zenglu@ict.ac.cn>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "hw/sysbus.h"
#include "qemu/timer.h"
#define DEBUG_LS1F_CAN
#define u8 unsigned char

#ifdef DEBUG_LS1F_CAN
#define dprintf(fs,...)					\
    fprintf(stderr,"ls1a_can: %s: "fs,__func__,##__VA_ARGS__)
#else
#define dprintf(fs,...)
#endif

typedef enum can_mode {
	STANDARD,
	EXTENDED,
} can_mode;

typedef struct ls1a_can_state {
    SysBusDevice busdev;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer *timer;
	u8 devno;
	u8 control;           
	u8 command;
	u8 status;           
	u8 intr;
	u8 ac;        
	u8 am;
	u8 timing0;        
	u8 timing1;        
	u8 __reserved[2];         
	u8 txbuf[10]; //off 10 (10--ID(10-3)  11--ID(2-0),RTR,DLC)
	u8 rxbuf[10]; //off 20 (20--ID(10-3)  21--ID(2-0),RTR,DLC)
	can_mode mode;
	int reset;
} ls1a_can_state;

typedef struct ls1a_can_data_t {
	u8 sendfrom;
	u8 data[10];
} ls1a_can_data_t;
static ls1a_can_data_t ls1a_can_data;
static u8 ls1a_can_cnt;

static int ls1a_can_reset(ls1a_can_state *s)
{
	int i;
	s->control=1;
	s->command=0;
	s->status=0;
	s->intr=0;
	s->ac=0;
	s->am=0;
	s->timing0=0;
	s->timing1=0;
	s->am=0;
	for(i=0;i<10;i++) {
		s->txbuf[i]=0;
		s->rxbuf[i]=0;
	}
	return 0;
}

static int ls1a_can_send(ls1a_can_state *s)
{
	int i;
	dprintf("QEMU:can%d sending...\n", s->devno);
	ls1a_can_data.sendfrom = s->devno;
	for(i = 0; i < 10; i++) {
		ls1a_can_data.data[i] = s->txbuf[i];
	}
	s->intr |= 0x2;
    qemu_irq_raise(s->irq);
	return 0;
}

static uint64_t ls1a_can_read(void *opaque, hwaddr offset, unsigned size)
{
	uint8_t val;
	ls1a_can_state *s = (ls1a_can_state *)opaque;
	offset&=0x3fff;

    switch (offset) {
    case 0x00: 
		val = s->control;
		break;
    case 0x01: 
		val = 0xff; //s->command;
		break;
    case 0x02: 
		val = s->status;
		break;
    case 0x03: 
		val = s->intr;
		break;
    case 0x04: 
		val = s->ac;
		break;
    case 0x05: 
		val = s->am;
		break;
    case 0x06: 
		val = s->timing0;
		break;
    case 0x07: 
		val = s->timing1;
		break;
    default:
		if(offset >= 10 && offset <=19)
			val = s->txbuf[offset-10];
		else if(offset >= 20 && offset <=29)
			val = s->rxbuf[offset-20];
		else 
			val = -1;
    }
	//dprintf("read offset=0x%llx, val=0x%x\n", offset, val);

	return val;
}

static void ls1a_can_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
	ls1a_can_state *s = (ls1a_can_state *)opaque;
	offset&=0x3fff;
	dprintf("CAN%d write offset=0x"TARGET_FMT_plx", val=0x"TARGET_FMT_plx"\n", s->devno, offset, val);

    switch (offset) {
    case 0x00: 
		if(val == 0x1)
			ls1a_can_reset(s);
		break;
    case 0x01: 
		if(val & 0x1) {
			ls1a_can_send(s);
		}
		break;
	case 0x03: 
		if(val & 0x1) {
			s->intr &= ~0x1;
			goto lower;
		}
		if(val & 0x2) {
			s->intr &= ~0x2;
			goto lower;
		}
		break;
lower:
		qemu_irq_lower(s->irq);
		break;
    case 0x04: 
		s->ac = val;
		break;
    case 0x05: 
		s->am = val;
		break;
    case 0x06: 
		s->timing0 = val;
		break;
    case 0x07: 
		s->timing1 = val;
		break;
    default:
		if(offset >= 10 && offset <=19)
			s->txbuf[offset-10] = val;
		else if(offset >= 20 && offset <=29)
			s->rxbuf[offset-20] = val;
    }
}

static void ls1a_can_recv_timer(void *vp)
{
	int i;
    ls1a_can_state *s = vp;
	if(s->devno != ls1a_can_data.sendfrom && ls1a_can_data.sendfrom != (u8)-1) {
		dprintf("CAN%d receiving from CAN%d...\n",s->devno, ls1a_can_data.sendfrom);
		for(i = 0; i < 10; i++)
			s->rxbuf[i] = ls1a_can_data.data[i];
		s->intr |= 0x1;
		ls1a_can_data.sendfrom = -1;
		qemu_irq_raise(s->irq);
	}

	int64_t timeout = (int64_t) get_ticks_per_sec()>>4;
	timer_mod(s->timer, qemu_clock_get_ms (QEMU_CLOCK_VIRTUAL) + timeout);
}


static const MemoryRegionOps ls1a_can_ops = {
    .read = ls1a_can_read,
    .write = ls1a_can_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

#define TYPE_SYS_BUS_LS1ACAN "ls1a_can"

#define SYS_BUS_LS1ACAN(obj) \
    OBJECT_CHECK(ls1a_can_state, (obj), TYPE_SYS_BUS_LS1ACAN)

static int ls1a_can_init(SysBusDevice *dev)
{
    ls1a_can_state *d = SYS_BUS_LS1ACAN(dev);

    memory_region_init_io(&d->iomem, NULL, &ls1a_can_ops, (void *)d, "ls1a i2c", 0x4000);
    ls1a_can_reset(d);

    sysbus_init_irq(dev, &d->irq);
    sysbus_init_mmio(dev, &d->iomem);

	d->devno = ls1a_can_cnt;
	ls1a_can_cnt++;
	
	ls1a_can_data.sendfrom=255;
        d->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, ls1a_can_recv_timer, d);
	int64_t timeout = (int64_t) SCALE_MS>>4;
	timer_mod(d->timer, qemu_clock_get_ms (QEMU_CLOCK_VIRTUAL) + timeout);

    return 0;
}


static void ls1a_can_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ls1a_can_init;
    dc->desc = "ls1a can";
}

static const TypeInfo ls1a_can_info = {
    .name          = "ls1a_can",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ls1a_can_state),
    .class_init    = ls1a_can_class_init,
};


static void ls1a_can_register_types(void)
{
    type_register_static(&ls1a_can_info);
}

type_init(ls1a_can_register_types)

