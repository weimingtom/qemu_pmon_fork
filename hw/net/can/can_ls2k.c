/*
 * CAN bus emulation for ls2k. 
 * 
 * Author: Chong Qiao <qiaochong@loongson.cn>
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

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "qemu/error-report.h"
#include "net/can_emu.h"
#include "qapi/error.h"
#include "can_sja1000.h"

typedef struct ls2k_can_state {
    SysBusDevice busdev;
    MemoryRegion iomem;
    qemu_irq irq;
    CanSJA1000State sja_state;
    CanBusState     *canbus;
} ls2k_can_state;

static uint64_t ls2k_can_read(void *opaque, hwaddr offset, unsigned size)
{
	ls2k_can_state *d = (ls2k_can_state *)opaque;
	CanSJA1000State *s = &d->sja_state;
	return can_sja_mem_read(s, offset, size);
}

static void ls2k_can_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
	ls2k_can_state *d = (ls2k_can_state *)opaque;
	CanSJA1000State *s = &d->sja_state;
	can_sja_mem_write(s, offset, val, size);
}

static const MemoryRegionOps ls2k_can_ops = {
    .read = ls2k_can_read,
    .write = ls2k_can_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

#define TYPE_SYS_BUS_LS2KCAN "ls2k_can"

#define SYS_BUS_LS2KCAN(obj) \
    OBJECT_CHECK(ls2k_can_state, (obj), TYPE_SYS_BUS_LS2KCAN)

static void ls2k_can_connect_irq_notifier(SysBusDevice *dev, qemu_irq irq){
    ls2k_can_state *d = SYS_BUS_LS2KCAN(dev);
    CanSJA1000State *s = &d->sja_state;
    can_sja_init(s, irq);
}

static int ls2k_can_init(SysBusDevice *dev)
{
    ls2k_can_state *d = SYS_BUS_LS2KCAN(dev);
    CanSJA1000State *s = &d->sja_state;

    memory_region_init_io(&d->iomem, NULL, &ls2k_can_ops, (void *)d, "ls1a i2c", 0x100);

    sysbus_init_mmio(dev, &d->iomem);
    sysbus_init_irq(dev, &d->irq);
    if (can_sja_connect_to_bus(s, d->canbus) < 0) {
        printf("can_sja_connect_to_bus failed");
        return -EFAULT;
    }
	return 0;
}

static void ls2k_can_instance_init(Object *obj)
{

    ls2k_can_state *d = SYS_BUS_LS2KCAN(obj);
    object_property_add_link((Object *)d, "canbus", TYPE_CAN_BUS,
                             (Object **)&d->canbus,
                             qdev_prop_allow_set_link_before_realize,
                             0, &error_abort);

}



static void ls2k_can_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ls2k_can_init;
    k->connect_irq_notifier = ls2k_can_connect_irq_notifier;
    dc->desc = "ls2k can";
}

static const TypeInfo ls2k_can_info = {
    .name          = "ls2k_can",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ls2k_can_state),
    .class_init    = ls2k_can_class_init,
    .instance_init = ls2k_can_instance_init,
};


static void ls2k_can_register_types(void)
{
    type_register_static(&ls2k_can_info);
}

type_init(ls2k_can_register_types)

