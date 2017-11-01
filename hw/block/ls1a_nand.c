#include "qemu/osdep.h"
#include "hw/hw.h"
#include "sysemu/block-backend.h"
#include "hw/qdev.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "sysemu/blockdev.h"
#include "exec/memory.h"
#include "sysemu/dma.h"
#define DPRINTF(a...) //printf(a)
#define HW_FLASH_H
static DeviceState *nand_init(BlockBackend *blk, int manf_id, int chip_id);
static void nand_setpins(DeviceState *dev, uint8_t cle, uint8_t ale,
                  uint8_t ce, uint8_t wp, uint8_t gnd);
#define nand_getpins ls1a_nand_getpins
#define nand_setio ls1a_nand_setio
#define nand_getio ls1a_nand_getio
#define nand_getbuswidth ls1a_nand_getbuswidth
void nand_getpins(DeviceState *dev, int *rb);
void nand_setio(DeviceState *dev, uint32_t value);
uint32_t nand_getio(DeviceState *dev);
uint32_t nand_getbuswidth(DeviceState *dev);
#include "nand.c"


struct dma_desc{
	uint32_t ordered;
	uint32_t saddr;
	uint32_t daddr;
	uint32_t length;
	uint32_t step_length;
	uint32_t step_times;
	uint32_t cmd;
	/*used by logic only*/
	uint32_t left;
	uint32_t active;
	uint32_t nextaddr;
};

enum {
DMA_INT_MASK =  1,
DMA_INT = 2,
DMA_SINGLE_TRANS_OVER = 4,
DMA_TRANS_OVER = 8,
DMA_ORDER_EN = 1,
DMA_READ_DDR = 0x1000,
};

struct ls1a_nand_regs{
uint32_t cmd;
uint32_t addr_l;
uint32_t addr_h;
uint32_t nand_timing;
uint32_t id_l;
uint32_t status_id_h;
uint32_t paramter;
uint32_t op_num;
uint32_t cs_rdy_map; /*0x20*/
uint32_t dma_address;/*0x40*/
};

enum {
NAND_CMD = 0,
NAND_ADDR_L = 4,
NAND_ADDR_H = 8,
NAND_NAND_TIMING = 0xc,
NAND_ID_L = 0x10,
NAND_STATUS_ID_H = 0x14,
NAND_NAND_PARAMETER = 0x18,
NAND_NAND_OP_NUM = 0x1c,
NAND_CS_RDY_MAP = 0x20,
PLL_8030 = 0x30, // not nand reg
PLL_8034 = 0x34, // not nand reg
NAND_DMA_ADDRESS = 0x40,
};



/*irq is placed on each module now */
typedef struct NandState{
	MemoryRegion iomem;
	MemoryRegion iomem1;
	struct ls1a_nand_regs regs;
	NANDFlashState *chip;
	struct dma_desc dma_desc;
	int dma_int_status;
	qemu_irq irq;
	union{
	AddressSpace *as;
	void *as_ptr;
	};
	uint8_t manf_id, chip_id;
} NandState;

typedef struct nand_sysbus_state {
	SysBusDevice busdev;
	NandState nand;
} nand_sysbus_state;



enum {
CMD_STOP = 0,
CMD_VALID = 1,
CMD_READ = 2,
CMD_WRITE = 4,
CMD_ERASE = 8,
CMD_BLKERASE = 16,
CMD_READID = 32,
CMD_RESET = 64,
CMD_READSTATUS = 128,
CMD_MAIN = 256,
CMD_SPARE = 512,
CMD_DONE = 1024,
};

static void nand_check_irq(NandState *s)
{
	/* edge interrupt */
	if(s->dma_int_status & 1)
	{
		qemu_irq_raise(s->irq);
		qemu_irq_lower(s->irq);
	}
	
	
	s->dma_int_status = 0;
}

/*
dma transfer size should not address len,but can less len.
*/
#define min(x,y) (((x)<(y))?(x):(y))

static int dma_next(NandState *s)
{

	if(!s->dma_desc.step_times)
	{
		if(s->dma_desc.cmd & DMA_INT_MASK)
		{
			s->dma_desc.cmd |= DMA_INT;
			s->dma_int_status = 1;
			nand_check_irq(s);
		}


		if(s->dma_desc.ordered & DMA_ORDER_EN)
		{
			dma_memory_read(s->as, s->dma_desc.ordered & ~DMA_ORDER_EN,(uint8_t *)&s->dma_desc,4*7);
			s->dma_desc.left = s->dma_desc.length * 4;
			s->dma_desc.step_times--;
		}
		else if(s->dma_desc.nextaddr)
		{
			dma_memory_read(s->as, s->dma_desc.nextaddr, (uint8_t *)&s->dma_desc,4*7);
			s->dma_desc.nextaddr = 0;
			s->dma_desc.left = s->dma_desc.length * 4;
		}
		else {
			s->dma_desc.cmd |= DMA_TRANS_OVER;
			s->dma_desc.active = 0;
		}
	}
	else
	{
		s->dma_desc.step_times--;
		s->dma_desc.saddr += s->dma_desc.step_length * 4;
		s->dma_desc.left = s->dma_desc.length * 4;
	}
	return s->dma_desc.active;
}



#define PAGE_ADDR(s) (s->regs.addr_h&0xffff)
#define COLUM_ADDR(s) (s->regs.addr_l)

static int nand_load_next(NandState *s)
{
	uint32_t offset,addr,iolen, oplen;
	int cmd;
	cmd = s->regs.cmd;

	/*nand addr maybe not page aligned*/
	if( (cmd & (CMD_SPARE |CMD_MAIN)) == CMD_SPARE)
	{
		addr = PAGE_ADDR(s) << s->chip->addr_shift;
		offset =  COLUM_ADDR(s);
		iolen = s->chip->oob_size - offset;
	}
	else if((cmd & (CMD_SPARE |CMD_MAIN)) == (CMD_SPARE|CMD_MAIN))
	{
		addr = PAGE_ADDR(s) << s->chip->addr_shift;
		offset =  COLUM_ADDR(s);
		iolen = (1 << s->chip->page_shift) + s->chip->oob_size - offset;
	}
	else
	{
		addr = PAGE_ADDR(s) << s->chip->addr_shift;
		offset =  COLUM_ADDR(s);
		iolen = (1 << s->chip->page_shift) - offset;
	}

	oplen = min(s->regs.op_num, iolen);

	s->chip->addr = addr;
	s->chip->blk_load(s->chip, addr, offset);
	s->chip->iolen = oplen;
	if(s->chip->iolen >= iolen)
	 s->regs.addr_h += 0x1;
	return oplen;
}


static int nand_write_next(NandState *s)
{
	uint32_t offset,addr,iolen, oplen;
	int cmd;
	cmd = s->regs.cmd;

again:

	/*nand addr maybe not page aligned*/
	if( (cmd & (CMD_SPARE |CMD_MAIN)) == CMD_SPARE)
	{
/*addr include 3 row addr and 2 col addr
 * one sectior size == pagesize + oobsize,
 */
		addr = PAGE_ADDR(s) << s->chip->addr_shift;
		offset =  COLUM_ADDR(s);
		iolen = s->chip->oob_size - offset;
	}
	else if((cmd & (CMD_SPARE |CMD_MAIN)) == (CMD_SPARE|CMD_MAIN))
	{
		addr = PAGE_ADDR(s) << s->chip->addr_shift;
		offset =  COLUM_ADDR(s);
		iolen = (1 << s->chip->page_shift) + s->chip->oob_size - offset;
	}
	else
	{
		addr = PAGE_ADDR(s) << s->chip->addr_shift;
		offset =  COLUM_ADDR(s);
		iolen = (1 << s->chip->page_shift) - offset;
	}

	oplen = min(s->regs.op_num, iolen);

	if(oplen && s->chip->iolen >= oplen)
	{
		s->chip->addr = addr;
		s->chip->offset = offset;
		s->chip->iolen = oplen;

		s->chip->blk_write(s->chip);

		s->chip->ioaddr = s->chip->io;
		s->chip->iolen = 0;
		if(s->chip->iolen >= iolen)
		s->regs.addr_h += 0x1;
		s->regs.op_num -= oplen;
		goto again;
	}

 return oplen - s->chip->iolen;
}


static int dma_readnand(NandState *s)
{
	uint32_t copied;

	while(s->dma_desc.active && s->regs.op_num && !(s->dma_desc.cmd & DMA_READ_DDR)  && (s->regs.cmd & CMD_VALID) && (s->regs.cmd &CMD_READ))
	{
		if(!s->chip->iolen)
	 	 nand_load_next(s);

		if(!s->dma_desc.left && !dma_next(s))
		 break;

		copied = min(s->dma_desc.left,s->chip->iolen);
		copied = min(copied,s->regs.op_num);

		if(!copied) break;

		dma_memory_write(s->as, s->dma_desc.saddr,s->chip->ioaddr,copied);

		s->chip->ioaddr += copied;
		s->chip->iolen -= copied;
		s->dma_desc.saddr += copied;
		s->dma_desc.left -= copied;
		s->regs.op_num -= copied;

		if(!s->dma_desc.left)
		 dma_next(s);

		if(!s->regs.op_num)
		{
			s->regs.cmd |= CMD_DONE;
			s->regs.cmd &= ~CMD_VALID;
			break;
		}

	}

	return s->dma_desc.active;
}


static int dma_writenand(NandState *s)
{
	uint32_t wantsize,copied;
	wantsize = 0;

	while(s->dma_desc.active && s->regs.op_num && (s->dma_desc.cmd & DMA_READ_DDR)  && (s->regs.cmd & CMD_VALID) && (s->regs.cmd &(CMD_READ|CMD_WRITE)))
	{
		if(!s->dma_desc.left && !dma_next(s))
		 break;

		if(!wantsize)
		 wantsize = nand_write_next(s);

		copied = min(s->dma_desc.left,wantsize);
		copied = min(copied, s->regs.op_num);

		if(!copied) break;

		dma_memory_read(s->as, s->dma_desc.saddr,s->chip->ioaddr,copied);

		s->chip->ioaddr += copied;
		s->chip->iolen += copied;
		s->dma_desc.saddr += copied;
		s->dma_desc.left -= copied;

		if(!s->dma_desc.left)
		 dma_next(s);

		 wantsize = nand_write_next(s);

		if(!s->regs.op_num)
		{
			s->regs.cmd |= CMD_DONE;
			s->regs.cmd &= ~CMD_VALID;
			break;
		}
	}

	return s->dma_desc.active;
}

static NandState  *ls1a_nand;
void ls1a_nand_set_dmaaddr(uint32_t val);

void ls1a_nand_set_dmaaddr(uint32_t val)
{
	NandState  *s = ls1a_nand;
	uint32_t dmaaddr;
	dmaaddr = val & ~0xf;

	if(val & 0x8)
	{
		if(s->dma_desc.active)
		{
		s->dma_desc.nextaddr = dmaaddr;
		}
		else
		{
		dma_memory_read(s->as, dmaaddr,(uint8_t *)&s->dma_desc,4*7);
		s->dma_desc.left = s->dma_desc.length * 4;
		s->dma_desc.step_times--;
		s->dma_desc.active = 1;
		s->dma_desc.nextaddr = 0;
		if(s->dma_desc.cmd & DMA_READ_DDR)
			dma_writenand(s);
		else
			dma_readnand(s);
		}
	}

	if(val & 0x4)
	{
		dma_memory_write(s->as, dmaaddr,(uint8_t *)&s->dma_desc,4*7);
	}

}


static void ls1a_nand_do_cmd(NandState *s,uint32_t cmd)
{
	unsigned int i;
	unsigned int  addr;
	s->regs.cmd = cmd & 0x3ff;

	if((cmd&CMD_VALID) == 0)
	{
		s->regs.cmd |= CMD_DONE|(0xf<<16);
		return ;
	}

	if(cmd & CMD_ERASE)
	{
		if(cmd & CMD_BLKERASE)
		{
			/*fix 1 now,should be s->chip->erase_shift
			 *
			 * erase_shfit + program_shift => linuxkernel erase_shit
			 */

			for(i=0,addr = PAGE_ADDR(s)<<s->chip->addr_shift; i < s->regs.op_num; i++,addr += 1<<(s->chip->addr_shift+s->chip->erase_shift))
			{
				s->chip->addr = addr;
				s->chip->blk_erase(s->chip);
			}

		}
		else
		{
			s->chip->addr = PAGE_ADDR(s)<<s->chip->addr_shift ;
			s->chip->blk_erase(s->chip);
		}

		s->regs.cmd |= CMD_DONE;
		s->regs.cmd &= ~CMD_VALID;
		s->regs.status_id_h = (s->regs.status_id_h&~0xff0000)|0xe00000;
	}
	else if(cmd & CMD_READID)
	{
		s->chip->cmd = NAND_CMD_READID;
		nand_command(s->chip);
		memcpy(&s->regs.id_l,s->chip->io,4);
		s->regs.id_l = s->chip->io[4]|(s->chip->io[1]<<24)|(s->chip->io[2]<<16)|(s->chip->io[3]<<8);
		s->regs.status_id_h = s->chip->io[0] | 0xe00000;
		s->regs.cmd |= CMD_DONE;
		s->regs.cmd &= ~CMD_VALID;
	}
	else if(cmd & NAND_CMD_READSTATUS)
	{
		s->chip->cmd = NAND_CMD_READSTATUS;
		nand_command(s->chip);
		s->regs.status_id_h &= 0xff;
		memcpy((char *)(&s->regs.status_id_h)+2,s->chip->io,1);
		s->regs.cmd |= CMD_DONE;
		s->regs.cmd &= ~CMD_VALID;
	}
	else if(cmd & CMD_READ)
	{
		s->chip->ioaddr = s->chip->io;
		s->chip->iolen = 0;
		dma_readnand(s);
	}
	else if(cmd & CMD_WRITE)
	{
		s->chip->ioaddr = s->chip->io;
		s->chip->iolen = 0;
		s->regs.status_id_h = (s->regs.status_id_h&~0xff0000)|0xe00000;
		dma_writenand(s);
	}
	else 
		s->regs.cmd |= CMD_DONE;
		s->regs.cmd |= 0xf<<16;

}


static void nand_nand_writel(void *ptr, hwaddr addr, uint64_t val, unsigned size)
{
	NandState *s = ptr;
	addr &= 0xff;
	switch(addr)
	{
	  case NAND_CMD:
		 ls1a_nand_do_cmd(s,val);
		break;
	  case NAND_ID_L:
		break;
	  case NAND_STATUS_ID_H:
		break;
	  case NAND_NAND_PARAMETER:
		break;
	  case NAND_DMA_ADDRESS:
		break;
	  default:
		*(int *)((void *)&s->regs + addr) = val;
		break;
	}

	DPRINTF("nand_nand_writel:  (addr 0x%08X), val 0x%08lX\n", (unsigned) addr, val);
}

static uint64_t nand_nand_readl(void *ptr, hwaddr addr, unsigned size)
{
	NandState *s = ptr;
	uint32_t val = 0;
	addr &= 0xff;
	switch(addr)
	{
	  default:
		val = *(int *)((void *)&s->regs + addr);
		break;
	}

	DPRINTF("nand_nand_readl:  (addr 0x%08X), val 0x%08X\n", (unsigned) addr, val);
	return val;
}


static const MemoryRegionOps ls1a_nand_ops = {
    .read = nand_nand_readl,
    .write = nand_nand_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

#define TYPE_SYS_BUS_LS1ANAND "ls1a_nand"

#define SYS_BUS_LS1ANAND(obj) \
    OBJECT_CHECK(nand_sysbus_state, (obj), TYPE_SYS_BUS_LS1ANAND)

static int ls1a_nand_init(SysBusDevice *dev)
{
    DriveInfo *nand;
    nand_sysbus_state *d = SYS_BUS_LS1ANAND(dev);
    //memset(&d->nand,0,sizeof(d->nand));
    nand = drive_get(IF_MTD, 0, 0);
    //d->nand.chip = (NANDFlashState *) nand_init(nand ? blk_by_legacy_dinfo(nand) : NULL, d->nand.manf_id?:0xec, d->nand.chip_id?:0xa1);
    d->nand.chip = (NANDFlashState *) nand_init(nand ? blk_by_legacy_dinfo(nand) : NULL, d->nand.manf_id?:0x2c, d->nand.chip_id?:0x48);
    nand_setpins(DEVICE(d->nand.chip), 0, 0, 0, 0, 0);

    if(!d->nand.as)
    d->nand.as = &address_space_memory;
    memory_region_init_io(&d->nand.iomem, NULL, &ls1a_nand_ops, (void *)&d->nand, "ls1a nand", 0x24);
    memory_region_init_io(&d->nand.iomem1, NULL, &ls1a_nand_ops, (void *)&d->nand, "ls1a nand", 0x4);
    sysbus_init_mmio(dev, &d->nand.iomem);
    sysbus_init_mmio(dev, &d->nand.iomem1);
    sysbus_init_irq(dev, &d->nand.irq);
    ls1a_nand = &d->nand;

    return 0;
}

static Property ls1a_nand_properties[] = {
    DEFINE_PROP_PTR("as", nand_sysbus_state, nand.as_ptr),
    DEFINE_PROP_UINT8("manufacturer_id", nand_sysbus_state, nand.manf_id, 0),
    DEFINE_PROP_UINT8("chip_id", nand_sysbus_state, nand.chip_id, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void ls1a_nand_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ls1a_nand_init;
    dc->desc = "ls1a nand";
    dc->props = ls1a_nand_properties;
}

static const TypeInfo ls1a_nand_info = {
    .name          = "ls1a_nand",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nand_sysbus_state),
    .class_init    = ls1a_nand_class_init,
};


static void ls1a_nand_register_types(void)
{
    type_register_static(&ls1a_nand_info);
}


type_init(ls1a_nand_register_types)
