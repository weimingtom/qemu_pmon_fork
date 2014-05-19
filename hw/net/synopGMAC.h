#include "hw/sysbus.h"
static inline DeviceState *
gmac_sysbus_create(NICInfo *nd, hwaddr base, qemu_irq irq)
{
    DeviceState *dev;

    dev = qdev_create(NULL, "sysbus-synopgmac");
    qdev_set_nic_properties(dev, nd);
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);
    return dev;
}
