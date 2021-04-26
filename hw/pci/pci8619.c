#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "qemu/timer.h"
#include "net/net.h"
#include "hw/sysbus.h"
#include "hw/loader.h"
#include <sys/mman.h>

#define DPRINTF(a...) //fprintf(stderr,a)
#define PCI8619_MEM_SIZE 0x04000000

typedef struct PCI8619State{
int mem;
uint8_t *mapaddr;
unsigned int mask;
qemu_irq irq;
MemoryRegion ram_vram;
uint16_t vendor_id, device_id;
        uint32_t map_offs;
        uint32_t map_len;
        char *mempath;
} PCI8619State;

typedef struct pci8619_pci_state {
    PCIDevice card;
    PCI8619State pci8619;
} pci8619_pci_state;


typedef struct pci8619_sysbus_state {
    SysBusDevice busdev;
    PCI8619State pci8619;
} pci8619_sysbus_state;


static PCI8619State *pci8619_new(PCI8619State *s)
{
        if (s->mempath) {
                int fd;
                fd=open(s->mempath,O_CREAT|O_RDWR, 0666);
                s->mapaddr = mmap(NULL,s->map_len,PROT_READ|PROT_WRITE|PROT_EXEC,MAP_SHARED,fd,s->map_offs);
                close(fd);
        } else {
                s->mapaddr = g_malloc0(s->map_len);
        }

	memory_region_init_ram_ptr(&s->ram_vram, NULL, "vram", s->map_len, s->mapaddr);
	s->mask = s->map_len -1;

	return s;
}


#define PCI8619_VENDOR_ID  0x16c3
#define PCI8619_DEVICE_ID  0xabcd

static void pci_pci8619_init(PCIDevice *dev, Error **errp)
{
    pci8619_pci_state * d = DO_UPCAST(pci8619_pci_state, card, dev);
    uint8_t *pci_conf;

    pci_conf = d->card.config;
    pci_conf[0x04] = 0x07; /* command = I/O space, Bus Master */
    pci_set_long(pci_conf + PCI_CLASS_REVISION, 0x068000ba);
    pci_conf[PCI_HEADER_TYPE] = PCI_HEADER_TYPE_NORMAL|PCI_HEADER_TYPE_MULTI_FUNCTION; /* header_type */
    pci_conf[0x3d] = 1;    /* interrupt pin 0 */
    pci_conf[0x34] = 0xdc;


    /* I/O handler for memory-mapped I/O */
    pci8619_new(&d->pci8619);


    pci_config_set_vendor_id(pci_conf, d->pci8619.vendor_id?:PCI8619_VENDOR_ID);
    pci_config_set_device_id(pci_conf, d->pci8619.device_id?:PCI8619_DEVICE_ID);

    pci_register_bar(&d->card, 2, PCI_BASE_ADDRESS_MEM_TYPE_64, &d->pci8619.ram_vram);

    //d->pci8619.irq = d->card.irq[0];
}


static Property pci8619_properties[] = {
    DEFINE_PROP_UINT16("vendor", pci8619_pci_state, pci8619.vendor_id,PCI8619_VENDOR_ID),
    DEFINE_PROP_UINT16("device", pci8619_pci_state, pci8619.device_id,PCI8619_DEVICE_ID),
    DEFINE_PROP_UINT32("offset", pci8619_pci_state, pci8619.map_offs,0x0f000000),
    DEFINE_PROP_UINT32("len", pci8619_pci_state, pci8619.map_len,0x01000000),
    DEFINE_PROP_STRING("mempath", pci8619_pci_state, pci8619.mempath),
    DEFINE_PROP_END_OF_LIST(),
};



static void pci8619_class_init(ObjectClass *klass, void *data)
{
    //DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    k->realize = pci_pci8619_init;
    k->romfile = "vgabios-vmware.bin";
    k->vendor_id = PCI8619_VENDOR_ID;
    k->device_id = PCI8619_DEVICE_ID;
    k->class_id = PCI_CLASS_MEMORY_RAM;
    dc->desc = "pci8619 Controller";
    dc->props = pci8619_properties;
}

static const TypeInfo pci8619_type_info = {
    .name          = "pci8619",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(pci8619_pci_state),
    .class_init    = pci8619_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};


static void pci8619_sysbus_register(void)
{
    type_register_static(&pci8619_type_info);
}

type_init(pci8619_sysbus_register)




//------------------------------

