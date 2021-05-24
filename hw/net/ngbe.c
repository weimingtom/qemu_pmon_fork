/*
 * QEMU synopsis gmac controller emulation
 *
 * Copyright (c) 2010 qiaochong@loongson.cn
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
#include "hw/hw.h"
#include "hw/pci/pci.h"
//#include "qemu-timer.h"
#include "net/net.h"
#include "hw/sysbus.h"
#include "hw/loader.h"

union emerald_descriptor {
	struct {
		/** Buffer address */
		uint64_t address;
		/** Length */
		uint16_t length;
		/** Flags */
		uint8_t flags;
		/** Command */
		uint8_t command;
		/** Status */
		uint32_t status;
	}tx;
	struct {
		/** Buffer address */
		uint64_t address;
		uint32_t status_error; /* ext status/error */
		uint16_t length; /* Packet length */
		uint16_t vlan; /* VLAN tag */		
	}rx;
} __attribute__ (( packed ));


/** Transmit Descriptor register block */
#define EMERALD_TD 0x03000UL

/** Receive Descriptor register block */
#define EMERALD_RD 0x01000UL

/** Device Control Register */
#define EMERALD_MIS_RST 0x1000CUL
#define EMERALD_MIS_RST_LAN0_RST	0x00000002UL	/**< lan0 reset */
#define EMERALD_MIS_RST_LAN1_RST	0x00000004UL	/**< lan1 reset */
#define EMERALD_MIS_RST_LAN2_RST	0x00000008UL	/**< lan2 reset */
#define EMERALD_MIS_RST_LAN3_RST	0x00000010UL	/**< lan3 reset */
#define EMERALD_MIS_ST 0x10028UL

#define EMERALD_PHY_CONFIG(_r)    (0x14000UL + ((_r) * 4))
#define EMERALD_INTPHY_PAGE_SELECT   31
#define EMERALD_MAC_TX_CFG                0x11000

#define EMERALD_PSR_CTL 0x15000UL

#define EMERALD_PSR_MAC_SWC_IDX 0x16210
#define EMERALD_PSR_MAC_SWC_AD_L  0x16200
#define EMERALD_PSR_MAC_SWC_AD_H  0x16204
#define EMERALD_PSR_MAC_SWC_VM    0x16208

#define EMERALD_DCTL_ENABLE	0x00000001UL	/**< Queue enable */
#define EMERALD_DCTL_RING_SIZE_SHIFT 1
#define EMERALD_DCTL_TX_WTHRESH_SHIFT 16
#define EMERALD_DCTL_RX_THER_SHIFT 16
#define EMERALD_DCTL_RX_HDR_SZ 0x0000F000UL
#define EMERALD_DCTL_RX_BUF_SZ 0x00000F00UL
#define EMERALD_DCTL_RX_SPLIT_MODE 0X04000000UL
#define EMERALD_DCTL_RX_BSIZEPKT_SHIFT          2 /* so many KBs */
#define EMERALD_DCTL_RX_BSIZEHDRSIZE_SHIFT      6
#define EMERALD_RX_HDR_SIZE 256
#define EMERALD_RX_BSIZE_DEFAULT 2048
#define EMERALD_PX_MISC_IEN 0x108
#define EMERALD_PX_MISC_IEN_ETH_LKDN 0x00000100UL
#define EMERALD_PX_MISC_IEN_ETH_LK   0x00040000UL 
#define NGBE_PX_INTA                           0x110
#define EMERALD_PX_ISB_ADDR_L 0x160
#define EMERALD_PX_ISB_ADDR_H 0x164
#define EMERALD_CFG_LAN_SPEED             0x14440

/** Receive/Transmit Descriptor Base Address Low (offset) */
#define EMERALD_DBAL 0x00

/** Receive/Transmit Descriptor Base Address High (offset) */
#define EMERALD_DBAH 0x04

/** Receive/Transmit Descriptor Head (offset) */
#define EMERALD_DH 0x08

/** Receive/Transmit Descriptor Tail (offset) */
#define EMERALD_DT 0x0C

#define EMERALD_DCTL 0x10

/** Interrupt Mask Set/Read Register */
#define EMERALD_PX_IMS0 0x140UL


/** Interrupt Mask Clear Register */
#define EMERALD_PX_IMC0 0x150UL
/** Descriptor done */
#define EMERALD_DESC_STATUS_DD 0x00000001UL
#define NGBE_RXD_STAT_EOP              0x00000002U /* End of Packet */
#define NGBE_MDIO_CLAUSE_SELECT                0x11220
#define NGBE_SPI_STATUS                0x1010C
#define NGBE_SPI_STATUS_FLASH_BYPASS   ((0x1) << 31)
#define NGBE_MDI_PHY_ID1_OFFSET                2
#define NGBE_MDI_PHY_ID2_OFFSET                3
#define NGBE_INTERNAL_PHY_ID                   0x000732
#define NGBE_MDI_PHY_ID_MASK                   0xFFFFFC00U
#define NGBE_DEVICE_CAPS_WOL_PORT0     0x8 /* WoL supported on port 0 */
#define NGBE_MIS_SWSM                  0x1002C
#define NGBE_MNG_SW_SM         0x1E004
#define NGBE_MNG_SW_SM_SM              0x00000001U /* software Semaphore */
#define NGBE_MIS_ST                    0x10028
#define NGBE_MIS_ST_MNG_INIT_DN        0x00000001U
#define NGBE_MNG_MBOX_CTL      0x1E044
#define NGBE_MNG_MBOX_CTL_FWRDY        0x4
#define NGBE_CFG_PORT_ST               0x14404
#define NGBE_MNG_MBOX          0x1E100
#define NGBE_CHECKSUM_CAP_ST_PASS      0x80658383
#define NGBE_PX_MISC_IC_PHY            0x00040000U /* link up */
#define NGBE_FLAG_NEED_LINK_UPDATE             (1 << 13)

#define DPRINTF(a...) //fprintf(stderr,a)
#define min(a,b) (((a)<(b))?(a):(b))

enum emerald_isb_idx {
	EMERALD_ISB_HEADER,
	EMERALD_ISB_MISC,
	EMERALD_ISB_VEC0,
	EMERALD_ISB_VEC1,
	EMERALD_ISB_MAX
};


typedef struct gmacMacRegisters              
{
        uint32_t tx_dbal, tx_dbah;
        uint32_t rx_dbah, rx_dbal;
        uint32_t tx_dh, tx_dt;
        uint32_t rx_dh, rx_dt;
        uint32_t inten;
        uint32_t intsts;
        uint32_t misc_inten;
        uint32_t px_isb_addr_l, px_isb_addr_h;
        uint32_t phy_regs[32];
        uint32_t isb_mem[EMERALD_ISB_MAX];
        uint32_t tx_dctl, rx_dctl;
        uint32_t mnw_sw_sm;
} gmacMacRegisters;

typedef struct gmacDmaRegisters             
{
} gmacDmaRegisters;

typedef struct GMACState{
MemoryRegion iomem;
QEMUTimer *timer;
    NICState *nic;
    NICConf conf;
gmacMacRegisters regs;
union{
	AddressSpace *as;
	void *as_ptr;
};
qemu_irq irq;
int receive_stop;
int buswidth;
int enh_desc;
} GMACState;

typedef struct ngbe_pci_state {
    PCIDevice dev;
    GMACState gmac;
} ngbe_pci_state;

typedef struct ngbe_sysbus_state {
    SysBusDevice busdev;
    GMACState gmac;
} ngbe_sysbus_state;

static void ngbe_transmit_demand(GMACState *s);
static void ngbe_receive_demand(GMACState *s);

static void ngbe_check_irq(GMACState *s)
{
	if(s->regs.intsts) {
		qemu_irq_raise(s->irq);
        }
	else qemu_irq_lower(s->irq);
}

static uint64_t ngbe_mem_readl(void *ptr, hwaddr addr, unsigned size)
{
	GMACState *s = ptr;
	uint32_t val = 0;
        switch (addr) {
        case NGBE_MNG_MBOX + 4:
                val = NGBE_CHECKSUM_CAP_ST_PASS;
                break;
        case NGBE_CFG_PORT_ST:
                val = 0x0;
                break;
        case NGBE_MNG_MBOX_CTL:
                val = NGBE_MNG_MBOX_CTL_FWRDY;
                break;
        case NGBE_MIS_ST:
                val = NGBE_MNG_SW_SM_SM;
                break;
        case  NGBE_MNG_SW_SM:
                val = s->regs.mnw_sw_sm;
                break;
        case NGBE_SPI_STATUS:
                val = NGBE_SPI_STATUS_FLASH_BYPASS;
                break;
        case EMERALD_PSR_MAC_SWC_AD_L:
                val = s->conf.macaddr.a[5]|(s->conf.macaddr.a[4] << 8) | (s->conf.macaddr.a[3] << 16) | (s->conf.macaddr.a[2] << 24);
                break;
        case EMERALD_PSR_MAC_SWC_AD_H:
                val = s->conf.macaddr.a[1]|(s->conf.macaddr.a[0] << 8);
                break;
        case NGBE_PX_INTA:
                val = s->regs.intsts;
                break;
        case EMERALD_TD + EMERALD_DBAH:
                val = s->regs.tx_dbah;
                break;
        case EMERALD_TD +  EMERALD_DBAL:
                val = s->regs.tx_dbal;
                break;
        case EMERALD_RD +  EMERALD_DBAH:
                val = s->regs.rx_dbah;
                break;
        case EMERALD_RD +  EMERALD_DBAL:
                val = s->regs.rx_dbal;
                break;
        case EMERALD_TD + EMERALD_DH:
                val = s->regs.tx_dh;
                break;
        case EMERALD_TD + EMERALD_DT:
                val = s->regs.tx_dt;
                break;
        case EMERALD_RD + EMERALD_DH:
                val = s->regs.rx_dh;
                break;
        case EMERALD_RD + EMERALD_DT:
                val = s->regs.rx_dt;
                break;
        case EMERALD_TD + EMERALD_DCTL:
                val = s->regs.tx_dctl;
                break;
        case EMERALD_RD + EMERALD_DCTL:
                val = s->regs.rx_dctl;
                break;
        case EMERALD_PX_ISB_ADDR_L:
                val = s->regs.px_isb_addr_l;
                break;
        case EMERALD_PX_ISB_ADDR_H:
                val = s->regs.px_isb_addr_h;
                break;
        case EMERALD_PHY_CONFIG(0) ... EMERALD_PHY_CONFIG(32) - 1:  
                val = s->regs.phy_regs[(addr - EMERALD_PHY_CONFIG(0x0))/4];
                if (addr == EMERALD_PHY_CONFIG(0x1a) && s->regs.phy_regs[31] == 0xa43)
                val = 0x1c;

                if (addr == EMERALD_PHY_CONFIG(0x1d) && s->regs.phy_regs[31] == 0xa43)
                val = NGBE_FLAG_NEED_LINK_UPDATE;
                if (addr == EMERALD_PHY_CONFIG(NGBE_MDI_PHY_ID1_OFFSET) && s->regs.phy_regs[31] == 0)
                        val = (NGBE_INTERNAL_PHY_ID  >> 6) & 0xffff;
                if (addr == EMERALD_PHY_CONFIG(NGBE_MDI_PHY_ID2_OFFSET) && s->regs.phy_regs[31] == 0)
                        val = (NGBE_INTERNAL_PHY_ID  << 10) & 0xffff;
                break;
        }


	DPRINTF("ngbe_mem_readl:  (addr 0x%08X), val 0x%08X\n", (unsigned) addr, val);

	return val;
}

static void ngbe_mem_writel(void *ptr, hwaddr addr, uint64_t val, unsigned size)
{
    GMACState *s = ptr;

        switch (addr) {
        case  NGBE_MNG_SW_SM:
                s->regs.mnw_sw_sm = val;
                break;
        case NGBE_MDIO_CLAUSE_SELECT:
                //s->regs.mdio_clause = val;
                break;
        case EMERALD_PSR_MAC_SWC_IDX:
                break;
        case NGBE_PX_INTA:
                s->regs.intsts &= ~val;
                ngbe_check_irq(s);
                break;
        case EMERALD_PX_MISC_IEN:
                s->regs.misc_inten = val;
                ngbe_check_irq(s);
                break;
        case EMERALD_PX_IMS0:
                s->regs.inten &= ~val;
                ngbe_check_irq(s);
                break;
        case EMERALD_PX_IMC0:
                s->regs.inten |= val;
                ngbe_check_irq(s);
                break;
        case EMERALD_TD + EMERALD_DBAH:
                s->regs.tx_dbah = val;
                break;
        case EMERALD_TD +  EMERALD_DBAL:
                s->regs.tx_dbal = val;
                break;
        case EMERALD_RD +  EMERALD_DBAH:
                s->regs.rx_dbah = val;
                break;
        case EMERALD_RD +  EMERALD_DBAL:
                s->regs.rx_dbal = val;
                break;
        case EMERALD_TD + EMERALD_DH:
                s->regs.tx_dh = val;
		ngbe_transmit_demand(s);//s->dma.DmaTxCurrDesc
                break;
        case EMERALD_TD + EMERALD_DT:
                s->regs.tx_dt = val;
                break;
        case EMERALD_RD + EMERALD_DH:
                s->regs.rx_dh = val;
		ngbe_receive_demand(s);
                break;
        case EMERALD_RD + EMERALD_DT:
                s->regs.rx_dt = val;
                break;
        case EMERALD_TD + EMERALD_DCTL:
                s->regs.tx_dctl = val;
                break;
        case EMERALD_RD + EMERALD_DCTL:
                s->regs.rx_dctl = val;
                break;
        case EMERALD_PX_ISB_ADDR_L:
                s->regs.px_isb_addr_l = val;
                break;
        case EMERALD_PX_ISB_ADDR_H:
                s->regs.px_isb_addr_h = val;
                break;
        case EMERALD_PHY_CONFIG(0) ... EMERALD_PHY_CONFIG(32) - 1:  
                s->regs.phy_regs[(addr - EMERALD_PHY_CONFIG(0x0))/4]  = val;
                break;
        }



    DPRINTF("ngbe_mem_writel:  (addr 0x%08X), val 0x%08X\n", (unsigned) addr, val);

}

static void ngbe_transmit_demand(GMACState *s)
{
        union emerald_descriptor desc;
        uint8_t txbuffer[0x4000];
        uint64_t tx_dt;
        int len;
        int dctl = s->regs.tx_dctl;
        int desccount =((dctl >> EMERALD_DCTL_RING_SIZE_SHIFT)&0xf)?  (128 * ((dctl >> EMERALD_DCTL_RING_SIZE_SHIFT)&0xf)) : 8192;
        if (!(dctl & EMERALD_DCTL_ENABLE) )
                return ; /*dma not start,ignore*/
        while (s->regs.tx_dh != s->regs.tx_dt)
        {
                tx_dt = s->regs.tx_dbal + ((uint64_t)s->regs.tx_dbah << 32) + s->regs.tx_dt*sizeof(union emerald_descriptor);
                dma_memory_read(s->as, tx_dt, &desc,sizeof(desc));
                len = desc.tx.length;
                if ( ! ( desc.tx.status &  ( EMERALD_DESC_STATUS_DD ) ) )
                {
                        //printf("desc.length=0x%08x\n", desc.length);

                        dma_memory_read(s->as, desc.tx.address, txbuffer, len);
                        qemu_send_packet(qemu_get_queue(s->nic), txbuffer, len);

                        desc.tx.status |=   EMERALD_DESC_STATUS_DD;
                        dma_memory_write(s->as, tx_dt, &desc, sizeof(desc));

                        s->regs.tx_dt = s->regs.tx_dt == desccount - 1? 0 : s->regs.tx_dt + 1;
                        s->regs.isb_mem[EMERALD_ISB_VEC0] = 1;
                        s->regs.isb_mem[EMERALD_ISB_MISC] = NGBE_PX_MISC_IC_PHY;
                        dma_memory_write(s->as, s->regs.px_isb_addr_l + ((uint64_t)s->regs.px_isb_addr_h << 32), s->regs.isb_mem, sizeof(s->regs.isb_mem));
                        s->regs.intsts |= 1;
                        ngbe_check_irq(s);

                }
                else
                {
                        break;
                }
        }
}

static void ngbe_receive_demand(GMACState *s)
{
	s->receive_stop = 0;
	qemu_flush_queued_packets(qemu_get_queue(s->nic));
}

static const MemoryRegionOps ngbe_ops = {
    .read = ngbe_mem_readl,
    .write = ngbe_mem_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static ssize_t ngbe_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
        GMACState *s = qemu_get_nic_opaque(nc);
        union emerald_descriptor desc;
        int dctl = s->regs.rx_dctl;
        int desccount =((dctl >> EMERALD_DCTL_RING_SIZE_SHIFT)&0xf)?  (128 * ((dctl >> EMERALD_DCTL_RING_SIZE_SHIFT)&0xf)) : 8192;
        uint64_t rx_dt;
        int mpsize = (dctl << EMERALD_DCTL_RX_BSIZEPKT_SHIFT) & (8192-1) & ~0x3ff;
        int count;


        if (!(dctl & EMERALD_DCTL_ENABLE) )
                return size; /*dma not start,ignore*/

        if (s->regs.rx_dh == s->regs.rx_dt)
                return size;

        rx_dt = s->regs.rx_dbal + ((uint64_t)s->regs.rx_dbah << 32) + s->regs.rx_dt*sizeof(union emerald_descriptor);
        dma_memory_read(s->as, rx_dt, &desc,sizeof(desc));

        if (  desc.rx.status_error &  ( EMERALD_DESC_STATUS_DD ) )
                return size;
        count = min(mpsize,size);
        dma_memory_write(s->as, desc.rx.address, buf, count);

        desc.rx.status_error |=   EMERALD_DESC_STATUS_DD | NGBE_RXD_STAT_EOP;
        desc.rx.length = count;
        desc.rx.vlan = 0;

        dma_memory_write(s->as, rx_dt, &desc, sizeof(desc));

        s->regs.rx_dt = s->regs.rx_dt == desccount - 1? 0 : s->regs.rx_dt + 1;
        s->regs.isb_mem[EMERALD_ISB_VEC0] = 1;
        s->regs.isb_mem[EMERALD_ISB_MISC] = NGBE_PX_MISC_IC_PHY;
        dma_memory_write(s->as, s->regs.px_isb_addr_l + ((uint64_t)s->regs.px_isb_addr_h << 32), s->regs.isb_mem, sizeof(s->regs.isb_mem));
        s->regs.intsts |= 1;
        ngbe_check_irq(s);


        return size;
}

static int ngbe_can_receive(NetClientState *nc)
{
    return 1;
}

static void ngbe_cleanup(NetClientState *nc)
{
    GMACState *s = qemu_get_nic_opaque(nc);

    s->nic = NULL;
}

static NetClientInfo net_ngbe_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = ngbe_can_receive,
    .receive = ngbe_receive,
    .cleanup = ngbe_cleanup,
};

static void ngbe_reg_init(GMACState *s)
{
	/*all regs default 0*/
}

static GMACState *ngbe_new(GMACState *s,const char *model, const char *name)
{
    ngbe_reg_init(s);
    memory_region_init_io(&s->iomem, NULL, &ngbe_ops, (void *)s, "synopgmac", 0x20000);

    qemu_macaddr_default_if_unset(&s->conf.macaddr);

    s->nic = qemu_new_nic(&net_ngbe_info, &s->conf,model,name, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);

	return s;
}


//-------------

/* PCI interface */
#define WXNGBE_VENDOR_ID  0x8088
#define WXNGBE_DEVICE_ID  0x0105

static void
pci_ngbe_uninit(PCIDevice *dev)
{
    ngbe_pci_state *d = DO_UPCAST(ngbe_pci_state, dev, dev);

    qemu_del_nic(d->gmac.nic);
}

static void pci_ngbe_init(PCIDevice *pci_dev, Error **errp)
{
    ngbe_pci_state *d = DO_UPCAST(ngbe_pci_state, dev, pci_dev);
    uint8_t *pci_conf;

    pci_conf = d->dev.config;

    pci_config_set_vendor_id(pci_conf, WXNGBE_VENDOR_ID);
    pci_config_set_device_id(pci_conf, WXNGBE_DEVICE_ID);
    pci_conf[0x04] = 0x07; /* command = I/O space, Bus Master */
    pci_config_set_class(pci_conf, PCI_CLASS_NETWORK_ETHERNET);
    pci_conf[PCI_HEADER_TYPE] = PCI_HEADER_TYPE_NORMAL|0x80; /* header_type */
    pci_conf[PCI_INTERRUPT_PIN] = 1; /* interrupt pin A */
    pci_conf[0x34] = 0xdc;


    ngbe_new(&d->gmac,object_get_typename(OBJECT(d)),d->dev.qdev.id); 
    if(!d->gmac.as) d->gmac.as = pci_get_address_space(pci_dev);
    d->gmac.irq = pci_allocate_irq(pci_dev);

    pci_register_bar(&d->dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->gmac.iomem);

}

static Property ngbe_pci_properties[] = {
    DEFINE_NIC_PROPERTIES(ngbe_pci_state, gmac.conf),
    DEFINE_PROP_PTR("as", ngbe_pci_state, gmac.as_ptr),
    DEFINE_PROP_END_OF_LIST(),
};

static void ngbe_pci_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = pci_ngbe_init;
    k->exit = pci_ngbe_uninit;
    k->vendor_id = WXNGBE_VENDOR_ID;
    k->device_id = WXNGBE_DEVICE_ID;
    k->revision = 0x03;
    k->class_id = PCI_CLASS_NETWORK_ETHERNET;
    dc->desc = "synopsis Gigabit Ethernet";
    dc->props = ngbe_pci_properties;
}

static const TypeInfo ngbe_pci_info = {
    .name          = "pci-wxgmac",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(ngbe_pci_state),
    .class_init    = ngbe_pci_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static void ngbe_pci_register_types(void)
{
    type_register_static(&ngbe_pci_info);
}

type_init(ngbe_pci_register_types)

//sysbus--

#define TYPE_SYS_BUS_WXGMAC "sysbus-synopgmac"

#define SYS_BUS_WXGMAC(obj) \
    OBJECT_CHECK(ngbe_sysbus_state, (obj), TYPE_SYS_BUS_WXGMAC)

static int ngbe_sysbus_init(SysBusDevice *dev)
{
    ngbe_sysbus_state *d = SYS_BUS_WXGMAC(dev);

    ngbe_new(&d->gmac,"synopgmac", DEVICE(dev)->id);
    if(!d->gmac.as) d->gmac.as = &address_space_memory;

    sysbus_init_irq(dev, &d->gmac.irq);
    sysbus_init_mmio(dev, &d->gmac.iomem);
   	

    return 0;
}

static Property ngbe_sysbus_properties[] = {
    DEFINE_NIC_PROPERTIES(ngbe_sysbus_state, gmac.conf),
    DEFINE_PROP_PTR("as", ngbe_sysbus_state, gmac.as_ptr),
    DEFINE_PROP_END_OF_LIST(),
};

static void ngbe_sysbus_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ngbe_sysbus_init;
    dc->desc = "synopsis Gigabit Ethernet";
    dc->props = ngbe_sysbus_properties;
}

static const TypeInfo ngbe_sysbus_info = {
    .name          = "sysbus-wxgmac",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ngbe_sysbus_state),
    .class_init    = ngbe_sysbus_class_init,
};


static void ngbe_sysbus_register_types(void)
{
    type_register_static(&ngbe_sysbus_info);
}

type_init(ngbe_sysbus_register_types)



