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
#define PCIRAM_MEM_SIZE 0x04000000

typedef struct PCIRAMState{
unsigned int mask;
qemu_irq irq;
uint16_t vendor_id, device_id;
uint32_t bar[6];
} PCIRAMState;

typedef struct pciram_pci_state {
    PCIDevice card;
    PCIRAMState pciram;
} pciram_pci_state;


typedef struct pciram_sysbus_state {
    SysBusDevice busdev;
    PCIRAMState pciram;
} pciram_sysbus_state;


#define PCIRAM_VENDOR_ID  0x16c3
#define PCIRAM_DEVICE_ID  0xabcd

static void pci_pciram_init(PCIDevice *dev, Error **errp)
{
    pciram_pci_state * d = DO_UPCAST(pciram_pci_state, card, dev);
    uint8_t *pci_conf;
    int i;

    pci_conf = d->card.config;
    pci_conf[0x04] = 0x07; /* command = I/O space, Bus Master */
    pci_config_set_class(pci_conf, PCI_CLASS_MEMORY_RAM);
    pci_conf[PCI_HEADER_TYPE] = PCI_HEADER_TYPE_NORMAL|PCI_HEADER_TYPE_MULTI_FUNCTION; /* header_type */
    pci_conf[0x3d] = 1;    /* interrupt pin 0 */
    pci_conf[0x34] = 0xdc;

    pci_config_set_vendor_id(pci_conf, d->pciram.vendor_id?:PCIRAM_VENDOR_ID);
    pci_config_set_device_id(pci_conf, d->pciram.device_id?:PCIRAM_DEVICE_ID);

    for(i=0;i<7;i++)
    {
       int io, size;
       if(!d->pciram.bar[i]) continue;
       io = d->pciram.bar[i]&1;
       if(io) size= ~(d->pciram.bar[i]&~3)+1;
       else size = ~(d->pciram.bar[i]&~0xf)+1;

	MemoryRegion *ram = g_new(MemoryRegion, 1);
	memory_region_init_ram_nomigrate(ram, NULL, "pciram.ram", size, NULL);
        
       pci_register_bar(&d->card, i, io?PCI_BASE_ADDRESS_SPACE_IO:PCI_BASE_ADDRESS_SPACE_MEMORY, ram);
    }

    if(d->pciram.bar[7]) 
	{
	*(int *)(pci_conf+0x90) = d->pciram.bar[7];
	pci_conf[0xd2] = 1;
	}

  //  d->pciram.irq = d->card.irq[0];
}


static Property pciram_properties[] = {
    DEFINE_PROP_UINT16("vendor", pciram_pci_state, pciram.vendor_id,PCIRAM_VENDOR_ID),
    DEFINE_PROP_UINT16("device", pciram_pci_state, pciram.device_id,PCIRAM_DEVICE_ID),
    DEFINE_PROP_UINT32("bar0", pciram_pci_state, pciram.bar[0],0),
    DEFINE_PROP_UINT32("bar1", pciram_pci_state, pciram.bar[1],0),
    DEFINE_PROP_UINT32("bar2", pciram_pci_state, pciram.bar[2],0),
    DEFINE_PROP_UINT32("bar3", pciram_pci_state, pciram.bar[3],0),
    DEFINE_PROP_UINT32("bar4", pciram_pci_state, pciram.bar[4],0),
    DEFINE_PROP_UINT32("bar5", pciram_pci_state, pciram.bar[5],0),
    DEFINE_PROP_UINT32("bar6", pciram_pci_state, pciram.bar[6],0),
    DEFINE_PROP_UINT32("bar7", pciram_pci_state, pciram.bar[7],0),
    DEFINE_PROP_END_OF_LIST(),
};



static void pciram_class_init(ObjectClass *klass, void *data)
{
    //DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    k->realize = pci_pciram_init;
    k->romfile = "vgabios-vmware.bin";
    k->vendor_id = PCIRAM_VENDOR_ID;
    k->device_id = PCIRAM_DEVICE_ID;
    k->class_id = PCI_CLASS_MEMORY_RAM;
    dc->desc = "pciram Controller";
    dc->props = pciram_properties;
}

static const TypeInfo pciram_type_info = {
    .name          = "pciram",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(pciram_pci_state),
    .class_init    = pciram_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};


static void pciram_sysbus_register(void)
{
    type_register_static(&pciram_type_info);
}

type_init(pciram_sysbus_register)




//------------------------------

