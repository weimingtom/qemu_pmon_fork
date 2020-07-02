/*
 * SPI flash emulation(st25p64v60). 
 * 
 * Author: Chong Qiao <qiaochong@ict.ac.cn>
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
	#include "sysemu/block-backend.h"
	#include "sysemu/blockdev.h"
	#include "hw/qdev.h"
	#include "hw/ssi/ssi.h"
	#include "hw/sd/sd.h"
	#include "qapi/error.h"
	#include "qemu-common.h"
	#include "exec/address-spaces.h"
	#include "hw/block/flash.h"

	//#define DEBUG_SPI_FLASH

	#ifdef DEBUG_SPI_FLASH
	#define dprintf(fs,...)					\
	    fprintf(stderr,"spi_nand: %s: "fs,__func__,##__VA_ARGS__)
	#else
	#define dprintf(fs,...)
	#endif
	#define FLASH_TYPE s->ftype
	#define FLASH_SIZE (spiflashs[FLASH_TYPE].mainsize + spiflashs[FLASH_TYPE].oobsize) * spiflashs[FLASH_TYPE].pages * spiflashs[FLASH_TYPE].blocks
	#define FLASH_MSIZE ((spiflashs[FLASH_TYPE].mainsize) * spiflashs[FLASH_TYPE].pages * spiflashs[FLASH_TYPE].blocks)
	#define SECTOR_SIZE (spiflashs[FLASH_TYPE].mainsize + spiflashs[FLASH_TYPE].oobsize)
	#define SECTOR_MSIZE (spiflashs[FLASH_TYPE].mainsize)
	#define FLASH_ID spiflashs[FLASH_TYPE].id
	#define FLASH_NID spiflashs[FLASH_TYPE].nid

	#define INFO(jedec_id, mainsize, oobsize, pages, block, flags) jedec_id, mainsize, oobsize, pages, block, flags

	#define TYPE_GD 1

	static struct spiflash_info 
	{
	const char *name;
	unsigned int id;
	unsigned int mainsize;
	unsigned int oobsize;
	unsigned int pages;
	unsigned int blocks;
	unsigned int flags;
	unsigned int nid;
	} spiflashs[] = {
		{ "GD5F1GQ4U", INFO(0xc8d100, 2048,  128,  64, 1024, 0) , 0xc8a1},
		{ "GD5F1GQ4R", INFO(0xc8c100, 2048,  128,  64, 1024, 0) , 0xc8a1},
		{ "GD5F1GQ4R", INFO(0xc8b468, 4096,  128,  64, 1024, TYPE_GD), 0xc8b4},
		{ "GD5F1GQ4R", INFO(0xc8a468, 4096,  128,  64, 1024, TYPE_GD), 0xc8b4},
	};

	typedef enum {
		SPI_FLASH_CMD,
		SPI_FLASH_READ_ADDR,
		SPI_RDM_READ_ADDR,
		SPI_FLASH_READ_DATA,
		SPI_REG_READ_AD,
		SPI_FLASH_READ_ID,
		SPI_FLASH_PP_AD,
		SPI_FLASH_PP_DATA,
		SPI_FLASH_SE_ADDR,
		SPI_FLASH_RES_DUMMY,
		SPI_FLASH_RES_DATA,
		SPI_FLASH_PPEXEC_AD,
	} spi_nand_mode;

	enum {
		FLASH_WREN = 6,
		FLASH_WRDI = 4,
		FLASH_RDID = 0x9f,
		FLASH_RDSR = 5,
		FLASH_WRSR = 1,
		FLASH_READ = 3,
		FLASH_FAST_READ = 0xb,
		FLASH_PP = 2,
		FLASH_SE = 0xd8,
		FLASH_RES = 0xAB,
	#define CMD_READ			0x13
	#define CMD_READ_RDM			0x03
	#define CMD_PROG_PAGE_CLRCACHE		0x02
	#define CMD_PROG_PAGE			0x84
	#define CMD_PROG_PAGE_EXC		0x10
	#define CMD_ERASE_BLK			0xd8
	#define CMD_WR_ENABLE			0x06
	#define CMD_WR_DISABLE			0x04
	#define CMD_READ_ID			0x9f
	#define CMD_RESET			0xff
	#define CMD_READ_REG			0x0f
	#define CMD_WRITE_REG			0x1f
	};

	# define NAND_CMD_READ0		0x00
	# define NAND_CMD_READ1		0x01
	# define NAND_CMD_READ2		0x50
	# define NAND_CMD_LPREAD2	0x30
	# define NAND_CMD_NOSERIALREAD2	0x35
	# define NAND_CMD_RANDOMREAD1	0x05
	# define NAND_CMD_RANDOMREAD2	0xe0
	# define NAND_CMD_READID	0x90
	# define NAND_CMD_RESET		0xff
	# define NAND_CMD_PAGEPROGRAM1	0x80
	# define NAND_CMD_PAGEPROGRAM2	0x10
	# define NAND_CMD_CACHEPROGRAM2	0x15
	# define NAND_CMD_BLOCKERASE1	0x60
	# define NAND_CMD_BLOCKERASE2	0xd0
# define NAND_CMD_READSTATUS	0x70
# define NAND_CMD_COPYBACKPRG1	0x85

# define ADDR_SHIFT		16
typedef struct SPIFlashState {
	SSISlave ssidev;
    	BlockBackend *blk;
	spi_nand_mode mode;
	unsigned char *buf;
	unsigned int size;
	int status;
	uint64_t addr;
	int ftype;
	int lastcmd;
	DeviceState *nand;
	uint8_t manf_id, chip_id;
	uint64_t caddr, raddr;
	int iolen;
	uint8_t iocache[8192+256];
} SPIFlashState;



static uint32_t spi_nand_transfer(SSISlave *dev, uint32_t val)
{
	SPIFlashState *s = FROM_SSI_SLAVE(SPIFlashState, dev);
	static int spi_addr;
	static int state_count;
	int val_data = val&0xff;
	int ret;

	dprintf("SPI FLASH TRANSFER %x mode=%d status=%d\n",(unsigned int)val,s->mode,s->status);

	if(dev->cs) {
		dprintf("CS disable\n");
		s->mode = SPI_FLASH_CMD;
		s->lastcmd = -1;
		return 0xff;
	} else {
		switch (s->mode) {
		case SPI_FLASH_CMD:
			dprintf("CS enable which cmd:%d\n",val_data);
			switch(val_data)
			{
				case CMD_READ_REG:
					s->lastcmd = val_data;
					s->mode = SPI_REG_READ_AD;
					spi_addr = 0;
					state_count = 0;
					break;
				case FLASH_FAST_READ:
				case CMD_READ_RDM: /*read from cache */
					s->lastcmd = val_data;
					s->mode = SPI_RDM_READ_ADDR;
					spi_addr = 0;
					state_count = 0;
					break;
				case CMD_READ: /*read page to cache */
					s->lastcmd = val_data;
					s->mode = SPI_FLASH_READ_ADDR;
					spi_addr = 0;
					state_count = 0;
					//NAND_CMD_READSTATUS
					break;
				case FLASH_WREN:
					s->status |= 0x2;
					s->mode = SPI_FLASH_CMD;
					break;
				case FLASH_WRDI:
					s->status &= ~0x2;
					s->mode = SPI_FLASH_CMD;
					break;
				case FLASH_RDID:
					state_count = 0;
					s->mode = SPI_FLASH_READ_ID;
					break;
				case CMD_PROG_PAGE_CLRCACHE:
					spi_addr = 0;
					state_count = 0;
					s->mode = SPI_FLASH_PP_AD;
					break;
				case CMD_PROG_PAGE_EXC:
					spi_addr = 0;
					state_count = 0;
					s->mode = SPI_FLASH_PPEXEC_AD;
					break;
				case CMD_ERASE_BLK:
					spi_addr = 0;
					state_count = 0;
					s->mode = SPI_FLASH_SE_ADDR;
					break;
			}
			return 0xff;
		case SPI_REG_READ_AD:
			/*0:cmd,1:addr,2:data*/
			state_count++;
			if(state_count == 2)
			{
				s->mode = SPI_FLASH_CMD;
				switch(spi_addr) {
				case 0xc0: /*status*/
					return 0;
				default:
					return 0;
				 
				}
			}
			spi_addr = (spi_addr<<8)|val_data;
			break;
		case SPI_FLASH_READ_ADDR:
			/*0-2:addr, high first*/
			dprintf("READ ADDR mode\n");
			state_count++;
			spi_addr = (spi_addr<<8)|val_data;
			if(state_count == 3)
			{
				s->raddr = spi_addr;
				/*here wait read cache*/
			}
			s->mode = SPI_FLASH_READ_ADDR;
			break;
		case SPI_RDM_READ_ADDR:
			/*0-2:addr, low first*/
			state_count++;
			spi_addr = (spi_addr<<8)|val_data;
			if(state_count == 3)
			{
				s->mode = SPI_FLASH_READ_DATA;
			if (spiflashs[FLASH_TYPE].flags & TYPE_GD) {
				state_count =  spi_addr & 0xffff;
			} else {
				state_count =  (spi_addr >> 8 ) & 0xffff;
			}
			}
			break;
		case SPI_FLASH_READ_DATA:
			ret = s->iocache[state_count++];
			s->mode = SPI_FLASH_READ_DATA;
			return ret;
			break;
		case SPI_FLASH_READ_ID:
			state_count++;
			switch(state_count) {
				case 1:
					return (FLASH_ID>>16)&0xff;
				case 2:
					return (FLASH_ID>>8)&0xff;
				case 3:
					return (FLASH_ID)&0xff;
				default:
					s->mode = SPI_FLASH_CMD;
					return 0xff;
			}
			break;
		case SPI_FLASH_PPEXEC_AD:
			/*0-2:addr*/
			state_count++;
			spi_addr = (spi_addr<<8)|val_data;
			if(state_count == 3)
			{
				s->raddr = spi_addr;
			}
			s->mode = SPI_FLASH_PPEXEC_AD;
			break;
		case SPI_FLASH_PP_AD:
			/*0-1:addr,2:data*/
			state_count++;
			spi_addr = (spi_addr<<8)|val_data;
			if(state_count == 2)
			{
				s->caddr = spi_addr;
				s->iolen = 0;
				state_count = 0;
				s->mode = SPI_FLASH_PP_DATA;
			}
			break;
		case SPI_FLASH_PP_DATA:
			s->iocache[s->iolen++] = val_data;
			state_count++;
			s->mode = SPI_FLASH_PP_DATA;
			return 0xff;
			break;
		case SPI_FLASH_SE_ADDR:
			state_count++;
			spi_addr = (spi_addr<<8)|val_data;
			if(state_count == 3)
			{
				s->raddr = spi_addr;
			}
			s->mode  = SPI_FLASH_SE_ADDR;
			break;
		case SPI_FLASH_RES_DUMMY:
			state_count++;
			if(state_count == 3)
			{
				s->mode = SPI_FLASH_RES_DATA;
			}
			break;
		case SPI_FLASH_RES_DATA:
			s->mode = SPI_FLASH_CMD;
			return 0x16;
			break;
		}

		/* Should never happen.  */
		return 0xff;
	}
}

#define CLE_0 0
#define CLE_1 1
#define ALE_0 0
#define ALE_1 1
#define CE_0  0
#define CE_1  0

static int spi_nand_cs(SSISlave *dev, bool select)
{
	SPIFlashState *s = FROM_SSI_SLAVE(SPIFlashState, dev);
	int i;
	unsigned long long naddr;
	dprintf("flash set cs %d\n", select);
	if(select) {
		/*cs raise*/
		switch (s->mode) {
			case SPI_FLASH_READ_DATA:
				nand_setpins(s->nand, CLE_0, ALE_0, CE_1, 1, 0);
				break;
			case SPI_FLASH_PPEXEC_AD:
				nand_setpins(s->nand, CLE_0, ALE_0, CE_1, 1, 0);
				nand_setpins(s->nand, CLE_1, ALE_0, CE_0, 1, 0);
				nand_setio(s->nand, NAND_CMD_PAGEPROGRAM1);
				nand_setpins(s->nand, CLE_0, ALE_1, CE_0, 1, 0);
				naddr = (s->raddr << ADDR_SHIFT)  + s->caddr;
				nand_setio(s->nand, naddr & 0xff);
				nand_setio(s->nand, (naddr>>8)&0xff);
				nand_setio(s->nand, (naddr>>16)&0xff);
				nand_setio(s->nand, (naddr>>24)&0xff);
				if (FLASH_MSIZE >= 256*0x100000)
					nand_setio(s->nand, (naddr>>32)&0xff);
				nand_setpins(s->nand, CLE_0, ALE_0, CE_0, 1, 0);
				for (i=0;i<s->iolen;i++)
					nand_setio(s->nand, s->iocache[i]);
				nand_setpins(s->nand, CLE_1, ALE_0, CE_0, 1, 0);
				nand_setio(s->nand, NAND_CMD_PAGEPROGRAM2);
				break;
			case SPI_FLASH_SE_ADDR:
				nand_setpins(s->nand, CLE_0, ALE_0, CE_1, 1, 0);
				nand_setpins(s->nand, CLE_1, ALE_0, CE_0, 1, 0);
				nand_setio(s->nand, NAND_CMD_BLOCKERASE1);
				nand_setpins(s->nand, CLE_0, ALE_1, CE_0, 1, 0);
				nand_setio(s->nand, s->raddr&0xff);
				nand_setio(s->nand, (s->raddr>>8)&0xff);
				nand_setio(s->nand, (s->raddr>>16)&0xff);
				nand_setpins(s->nand, CLE_0, ALE_0, CE_0, 1, 0);
				nand_setpins(s->nand, CLE_1, ALE_0, CE_0, 1, 0);
				nand_setio(s->nand, NAND_CMD_BLOCKERASE2);
				nand_setpins(s->nand, CLE_0, ALE_0, CE_1, 1, 0);
				break;
			case SPI_FLASH_READ_ADDR:
				nand_setpins(s->nand, CLE_0, ALE_0, CE_1, 1, 0);
				nand_setpins(s->nand, CLE_1, ALE_0, CE_0, 1, 0);
				nand_setio(s->nand, NAND_CMD_READ0);
				nand_setpins(s->nand, CLE_0, ALE_1, CE_0, 1, 0);
				naddr = s->raddr << ADDR_SHIFT;
				nand_setio(s->nand, naddr & 0xff);
				nand_setio(s->nand, (naddr>>8)&0xff);
				nand_setio(s->nand, (naddr>>16)&0xff);
				nand_setio(s->nand, (naddr>>24)&0xff);
				if (FLASH_MSIZE >= 256*0x100000)
					nand_setio(s->nand, (naddr>>32)&0xff);
				nand_setpins(s->nand, CLE_0, ALE_0, CE_0, 1, 0);
				for (i=0; i< SECTOR_SIZE; i++)
					s->iocache[i] = nand_getio(s->nand);
				nand_setpins(s->nand, CLE_0, ALE_0, CE_1, 1, 0);
				break;
			default:
				break;
		}
		s->mode = SPI_FLASH_CMD;
	}
	return 0;
}

static Property spi_nand_properties[] = {
    DEFINE_PROP_DRIVE("drive", SPIFlashState, blk),
    DEFINE_PROP_INT32("ftype", SPIFlashState, ftype, 0),
    DEFINE_PROP_END_OF_LIST(),
};


static void spi_nand_realize(SSISlave *dev, Error **errp)
{
	SPIFlashState *s = FROM_SSI_SLAVE(SPIFlashState, dev);
	DriveInfo *spinand = drive_get(IF_MTD, 0, 1);
	s->nand = nand_init(blk_by_legacy_dinfo(spinand), FLASH_NID >>8, FLASH_NID & 0xff);

	s->mode = SPI_FLASH_CMD;
}

static void spi_nand_class_init(ObjectClass *klass, void *data)
{
    SSISlaveClass *k = SSI_SLAVE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    k->realize = spi_nand_realize;
    k->transfer = spi_nand_transfer;
    k->set_cs = spi_nand_cs;
    k->cs_polarity = SSI_CS_LOW;
    dc->props = spi_nand_properties;
}


static const TypeInfo spi_nand_info = {
    .name          = "spi-nand",
    .parent        = TYPE_SSI_SLAVE,
    .instance_size = sizeof(SPIFlashState),
    .class_init    = spi_nand_class_init,
};

static void spi_nand_register_types(void)
{
    type_register_static(&spi_nand_info);
}

type_init(spi_nand_register_types)

