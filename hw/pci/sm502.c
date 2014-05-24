#include "hw.h"
#include "sysbus.h"
#include "pci/pci.h"
#include "sysemu/sysemu.h"
#include "sysemu/dma.h"
#include "qemu/range.h"
#include "qdev-addr.h"

#define PCI_VENDOR_ID_SM502 0x126f
#define PCI_DEVICE_ID_SM502 0x501

void *sm502_new(uint32_t local_mem_bytes, qemu_irq irq,
                CharDriverState *chr, MemoryRegion *memcontainer, MemoryRegion *iocontainer, DeviceState **pusbdev);


typedef struct SM502PciState {
    PCIDevice card;
    void *sm502;
    void *serial;
    DeviceState *usbdev;
    MemoryRegion memcontainer;
    MemoryRegion iocontainer;
} SM502PciState;


int pci_sm502_init(PCIBus *bus, int devfn, CharDriverState *chr);
int pci_sm502_init(PCIBus *bus, int devfn, CharDriverState *chr)
{
    PCIDevice *dev;

    dev = pci_create(bus, devfn, "sm502");
    qdev_prop_set_ptr(&dev->qdev, "serial", chr);
    qdev_init_nofail(&dev->qdev);

    return 0;
}

static int sm502_initfn(PCIDevice *dev)
{
    struct SM502PciState *d;
    d = DO_UPCAST(struct SM502PciState, card, dev);
    pci_config_set_vendor_id(d->card.config,0x126f);
    pci_config_set_device_id(d->card.config,0x0501);
    d->card.config[PCI_COMMAND] = PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
                                  PCI_COMMAND_MASTER;

    pci_config_set_class(d->card.config,PCI_CLASS_DISPLAY_OTHER);
//    pci_set_byte(&d->card.config[PCI_CLASS_PROG], SM502_PROGMODE_MAJOR_REV_1);

    d->card.config[PCI_CACHE_LINE_SIZE] = 0x08;  /* Cache line size */
    d->card.config[PCI_LATENCY_TIMER]   = 0x00;  /* Latency timer */
    d->card.config[PCI_HEADER_TYPE]     = PCI_HEADER_TYPE_NORMAL;
    d->card.config[PCI_INTERRUPT_PIN]   = 1;     /* interrupt pin 0 */

    d->sm502 = sm502_new(0x2000000, d->card.irq[0],d->serial, &d->memcontainer, &d->iocontainer, &d->usbdev);

    pci_register_bar(&d->card, 0, PCI_BASE_ADDRESS_SPACE_MEMORY,
                     &d->memcontainer);
    pci_register_bar(&d->card, 1,
                           PCI_BASE_ADDRESS_SPACE_MEMORY, &d->iocontainer);
    return 0;
}


static Property sm502_properties[] = {
    DEFINE_PROP_PTR("serial", SM502PciState, serial),
    DEFINE_PROP_END_OF_LIST(),
};

static void sm502_write_config(PCIDevice *dev, uint32_t addr,
                                      uint32_t val, int l)
{
    struct SM502PciState *d;
    d = DO_UPCAST(struct SM502PciState, card, dev);

    pci_default_write_config(dev, addr, val, l);

    if (!range_covers_byte(addr, l, PCI_BASE_ADDRESS_0)) {
        return;
    }

    d->usbdev->realized = 0;
    qdev_prop_set_taddr(d->usbdev, "localmem_base", val);
    d->usbdev->realized = 1;
}

static void sm502_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->init = sm502_initfn;
    k->config_write = sm502_write_config;
    k->vendor_id = PCI_VENDOR_ID_SM502;
    k->device_id = PCI_DEVICE_ID_SM502;
    k->class_id = 0x3800;
    k->revision = 0xc0;
    dc->desc = "sm502 pci card";
    dc->no_user = 1;
    dc->props = sm502_properties;
}

static const TypeInfo sm502_info = {
    .name          = "sm502",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(SM502PciState),
    .class_init    = sm502_class_init,
};

static void sm502_register_types(void)
{
    type_register_static(&sm502_info);
}

type_init(sm502_register_types)
