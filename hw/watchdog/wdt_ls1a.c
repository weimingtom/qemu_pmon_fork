/*
 * Virtual hardware watchdog for ls1a. 
 * 
 * Author: Zeng Lu <zenglu@loongson.cn>
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

#include "qemu-common.h"
#include "hw/sysbus.h"
#include "sysemu/watchdog.h"
#include "hw/hw.h"
#include "hw/isa/isa.h"
#include "hw/i386/pc.h"
#include "qemu/timer.h"

//#define DEBUG_LS1F_WDT

#ifdef DEBUG_LS1F_WDT
#define dprintf(fs,...)					\
    fprintf(stderr,"ls1a_wdt: %s: "fs,__func__,##__VA_ARGS__)
#else
#define dprintf(fs,...)
#endif

typedef struct ls1a_wdt_state {
    SysBusDevice busdev;
    MemoryRegion iomem;
    QEMUTimer *timer;
    uint32_t wdt_en;
    uint32_t wdt_set;
    uint32_t wdt_timer;
} ls1a_wdt_state;

/* This is the timer.  We use a global here because the watchdog
 * code ensures there is only one watchdog (it is located at a fixed,
 * unchangable IO port, so there could only ever be one anyway).
 */

/* This is called when the watchdog expires. */
static void ls1a_wdt_timer_expired(void *vp)
{
    ls1a_wdt_state *s = vp;

    dprintf("watchdog expired\n");

    watchdog_perform_action();
    timer_del(s->timer);
}


static uint64_t ls1a_wdt_read(void *opaque, hwaddr offset, unsigned size)
{
    ls1a_wdt_state *s = (ls1a_wdt_state *)opaque;
	offset&=0xf;

    switch (offset) {
    case 0x00: 
		return s->wdt_en;
    case 0x08: 
		return s->wdt_set;
    case 0x04: 
		return s->wdt_timer;
    default:
        return 0;
    }
}

static void ls1a_wdt_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    ls1a_wdt_state *s = (ls1a_wdt_state *)opaque;
	offset&=0xf;
	dprintf("write offset=0x"TARGET_FMT_plx", val=0x"TARGET_FMT_plx"\n", offset, value);

    switch (offset) {
    case 0x00: 
		s->wdt_en = value&1;
		if(s->wdt_en==0)
			timer_del(s->timer);
		break;
    case 0x08: 
		if(s->wdt_en&0x1) {
			s->wdt_set = value&1;
			if(s->wdt_set == 1) {
				int64_t timeout = (int64_t) s->wdt_timer;
				timer_mod(s->timer, qemu_clock_get_ms (QEMU_CLOCK_VIRTUAL) + timeout);
			}
			s->wdt_set = 0;
		}
		break;
    case 0x04: 
		s->wdt_timer = value;
		break;
    default:
		;
    }
}

static WatchdogTimerModel model = {
    .wdt_name = "ls1a_wdt",
    .wdt_description = "Watchdog for LS1F",
};


static const MemoryRegionOps ls1a_wdt_ops = {
    .read = ls1a_wdt_read,
    .write = ls1a_wdt_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


#define TYPE_SYS_BUS_LS1AWDT "ls1a_wdt"

#define SYS_BUS_LS1AWDT(obj) \
    OBJECT_CHECK(ls1a_wdt_state, (obj), TYPE_SYS_BUS_LS1AWDT)

static int ls1a_wdt_init(SysBusDevice *dev)
{
    ls1a_wdt_state *d = SYS_BUS_LS1AWDT(dev);
#ifdef DEBUG_LS1F_WDT
	select_watchdog_action("debug");
#endif

    memory_region_init_io(&d->iomem, NULL, &ls1a_wdt_ops, (void *)d, "ls1a wdt", 0xc);

    sysbus_init_mmio(dev, &d->iomem);
    d->timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, ls1a_wdt_timer_expired, d);

    return 0;
}


static void ls1a_wdt_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ls1a_wdt_init;
    dc->desc = "ls1a wdt";
}

static const TypeInfo ls1a_wdt_info = {
    .name          = "ls1a_wdt",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ls1a_wdt_state),
    .class_init    = ls1a_wdt_class_init,
};


static void ls1a_wdt_register_types(void)
{
    watchdog_add_model(&model);
    type_register_static(&ls1a_wdt_info);
}

type_init(ls1a_wdt_register_types)
