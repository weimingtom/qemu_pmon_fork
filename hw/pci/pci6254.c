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
#define PCI6254_MEM_SIZE 0x04000000

typedef struct PCI6254State{
int fd;
int mem;
uint8_t *mapaddr;
unsigned int mask;
qemu_irq irq;
MemoryRegion ram_vreg;
MemoryRegion ram_vram;
uint16_t vendor_id, device_id;
} PCI6254State;

typedef struct pci6254_pci_state {
    PCIDevice card;
    PCI6254State pci6254;
} pci6254_pci_state;


typedef struct pci6254_sysbus_state {
    SysBusDevice busdev;
    PCI6254State pci6254;
} pci6254_sysbus_state;


static void pci6254_reg_init(PCI6254State *s)
{
}

static PCI6254State *pci6254_new(PCI6254State *s)
{
	int fd;
	fd=open("/tmp/shm",O_CREAT|O_RDWR, 0666);
	if(ftruncate(fd, 0x10010000))
         printf("truncate failed\n");
	pci6254_reg_init(s);

	s->mapaddr = mmap(NULL,PCI6254_MEM_SIZE,PROT_READ|PROT_WRITE|PROT_EXEC,MAP_SHARED,fd,0);

	memory_region_init_ram_ptr(&s->ram_vreg, NULL, "vreg",  PCI6254_MEM_SIZE, s->mapaddr);
	memory_region_init_ram_ptr(&s->ram_vram, NULL, "vram",  PCI6254_MEM_SIZE, s->mapaddr+0x1000);
	s->mask = PCI6254_MEM_SIZE -1;
	s->fd = fd;

	return s;
}


#define PCI6254_VENDOR_ID  0x16c3
#define PCI6254_DEVICE_ID  0xabcd

static void pci_pci6254_init(PCIDevice *dev, Error **errp)
{
    pci6254_pci_state * d = DO_UPCAST(pci6254_pci_state, card, dev);
    uint8_t *pci_conf;

    pci_conf = d->card.config;
    pci_conf[0x04] = 0x07; /* command = I/O space, Bus Master */
    pci_config_set_class(pci_conf, PCI_CLASS_MEMORY_RAM);
    pci_conf[PCI_HEADER_TYPE] = PCI_HEADER_TYPE_NORMAL; /* header_type */
    pci_conf[0x3d] = 1;    /* interrupt pin 0 */
    pci_conf[0x34] = 0xdc;


    /* I/O handler for memory-mapped I/O */
    pci6254_new(&d->pci6254);


    pci_config_set_vendor_id(pci_conf, d->pci6254.vendor_id?:PCI6254_VENDOR_ID);
    pci_config_set_device_id(pci_conf, d->pci6254.device_id?:PCI6254_DEVICE_ID);

    pci_register_bar(&d->card, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->pci6254.ram_vreg);
    if(d->pci6254.vendor_id == 0x1022)
    pci_register_bar(&d->card, 1, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->pci6254.ram_vram);
    else
    pci_register_bar(&d->card, 2, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->pci6254.ram_vram);

    //d->pci6254.irq = d->card.irq[0];
}


static Property pci6254_properties[] = {
    DEFINE_PROP_UINT16("vendor", pci6254_pci_state, pci6254.vendor_id,PCI6254_VENDOR_ID),
    DEFINE_PROP_UINT16("device", pci6254_pci_state, pci6254.device_id,PCI6254_DEVICE_ID),
    DEFINE_PROP_END_OF_LIST(),
};



static void pci6254_class_init(ObjectClass *klass, void *data)
{
    //DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    k->realize = pci_pci6254_init;
    k->romfile = "vgabios-vmware.bin";
    k->vendor_id = PCI6254_VENDOR_ID;
    k->device_id = PCI6254_DEVICE_ID;
    k->class_id = PCI_CLASS_MEMORY_RAM;
    dc->desc = "pci6254 Controller";
    dc->props = pci6254_properties;
}

static const TypeInfo pci6254_type_info = {
    .name          = "pci6254",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(pci6254_pci_state),
    .class_init    = pci6254_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};


static void pci6254_sysbus_register(void)
{
    type_register_static(&pci6254_type_info);
}

type_init(pci6254_sysbus_register)




//------------------------------

