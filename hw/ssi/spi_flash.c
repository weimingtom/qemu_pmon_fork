/*
 * SPI flash emulation(st25p64v60). 
 * 
 * Author: Zeng Lu <zenglu@ict.ac.cn>
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

//#define DEBUG_SPI_FLASH

#ifdef DEBUG_SPI_FLASH
#define dprintf(fs,...)					\
    fprintf(stderr,"spi_flash: %s: "fs,__func__,##__VA_ARGS__)
#else
#define dprintf(fs,...)
#endif

typedef enum {
	SPI_FLASH_CMD,

	SPI_FLASH_READ_ADDR,
	SPI_FLASH_READ_ADDR_FAST,
	SPI_FLASH_READ_DATA,
	//SPI_FLASH_READ_DATA_FAST,

	SPI_FLASH_READ_ID,

	SPI_FLASH_READ_SR,
	SPI_FLASH_WRITE_SR,

	SPI_FLASH_PP_ADDR,
	SPI_FLASH_PP_DATA,

	SPI_FLASH_SE_ADDR,

	SPI_FLASH_RES_DUMMY,
	SPI_FLASH_RES_DATA,
} spi_flash_mode;

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
	FLASH_BE = 0xC7,
	FLASH_RES = 0xAB,
};

typedef struct SPIFlashState {
	SSISlave ssidev;
    	BlockBackend *blk;
	spi_flash_mode mode;
	unsigned char *buf;
	unsigned int size;
	int status;
	MemoryRegion mem;
	MemoryRegion mem1;
	uint64_t addr;
	int isram;
} SPIFlashState;

static void program_page(SPIFlashState *s, int addr, unsigned char value, int count)
{
	uint8_t pp_buf[512];
	s->buf[addr] = value;
	if(s->blk)
	{
		blk_pread(s->blk, addr&~0x1ff, pp_buf, 1<<BDRV_SECTOR_BITS);
		pp_buf[addr&0x1ff] = value;
		blk_pwrite(s->blk, addr&~0x1ff, pp_buf, 1<<BDRV_SECTOR_BITS, 0);
	}
}

static void sector_erase(SPIFlashState *s, int addr)
{
	unsigned char * buf = g_malloc0(0x10000);
	memset(buf, -1, 0x10000);
	int sec_addr = addr & 0xff0000;
	memcpy(s->buf+sec_addr, buf, 0x10000);
	if(s->blk)
		blk_pwrite(s->blk, sec_addr, buf, 128<<BDRV_SECTOR_BITS, 0);
	g_free(buf);
}

static void bulk_erase(SPIFlashState *s)
{
	int sector;
	for(sector = 0; sector <= 0x7f; sector++) {
		sector_erase(s, sector << 16);
	}
}

static uint32_t spi_flash_transfer(SSISlave *dev, uint32_t val)
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
		return 0xff;
	} else {
		switch (s->mode) {
		case SPI_FLASH_CMD:
			dprintf("CS enable which cmd:%d\n",val_data);
			switch(val_data)
			{
				case FLASH_READ:
					s->mode = SPI_FLASH_READ_ADDR;
					spi_addr = 0;
					state_count = 0;
					break;
				case FLASH_FAST_READ:
					s->mode = SPI_FLASH_READ_ADDR_FAST;
					spi_addr = 0;
					state_count = 0;
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
				case FLASH_RDSR:
					s->mode = SPI_FLASH_READ_SR;
					break;
				case FLASH_WRSR:
					s->mode = SPI_FLASH_WRITE_SR;
					break;
				case FLASH_PP:
					spi_addr = 0;
					state_count = 0;
					s->mode = SPI_FLASH_PP_ADDR;
					break;
				case FLASH_SE:
					s->mode = SPI_FLASH_SE_ADDR;
					break;
				case FLASH_BE:
					bulk_erase(s);
					s->mode = SPI_FLASH_CMD;
					break;
				case FLASH_RES:
					state_count = 0;
					s->mode = SPI_FLASH_RES_DUMMY;
					break;
			}
			return 0xff;
		case SPI_FLASH_READ_ADDR:
			dprintf("READ ADDR mode\n");
			state_count++;
			spi_addr = (spi_addr<<8)|val_data;
			if(state_count == 3)
			{
				state_count = spi_addr;
				s->mode = SPI_FLASH_READ_DATA;
			}
			break;
		case SPI_FLASH_READ_ADDR_FAST:
			state_count++;
			if(state_count <= 3)
			  spi_addr = (spi_addr<<8)|val_data;
			else if(state_count == 4) {
				;//dummy
				state_count = spi_addr;
				s->mode = SPI_FLASH_READ_DATA;
			}
			break;
		case SPI_FLASH_READ_DATA:
			dprintf("READ DATA mode,addr:0x%x\n",state_count);
			if(state_count >= 0x7fffff) {
				state_count = 0;
			}
			ret = s->buf[state_count];
			state_count++;
			s->mode = SPI_FLASH_READ_DATA;
			return ret;
			break;
		case SPI_FLASH_READ_ID:
			state_count++;
			switch(state_count) {
				case 1:
					return 0x20;
				case 2:
					return 0x20;
				case 3:
					return 0x17;
				default:
					s->mode = SPI_FLASH_CMD;
					return 0xff;
			}
			break;
		case SPI_FLASH_READ_SR:
			return s->status;
			s->mode = SPI_FLASH_CMD;
			break;
		case SPI_FLASH_WRITE_SR:
			s->status |= val_data|0x9c;
			s->mode = SPI_FLASH_CMD;
			break;
		case SPI_FLASH_PP_ADDR:
			state_count++;
			spi_addr = (spi_addr<<8)|val_data;
			if(state_count == 3)
			{
				state_count = 1;
				s->mode = SPI_FLASH_PP_DATA;
				dprintf("PP ADDR %x\n", spi_addr);
			}
			break;
		case SPI_FLASH_PP_DATA:
			dprintf("PP DATA mode addr:%x\n", spi_addr);
			program_page(s, spi_addr,val_data, state_count);
			if((spi_addr & 0xff) == 0xff)
				spi_addr &= ~0xff;
			else
				spi_addr += 1;
			state_count++;
			s->mode = SPI_FLASH_PP_DATA;
			return 0xff;
			break;
		case SPI_FLASH_SE_ADDR:
			state_count++;
			spi_addr = (spi_addr<<8)|val_data;
			if(state_count == 3)
			{
				//sector erase
				sector_erase(s, spi_addr);
			}
			s->mode = SPI_FLASH_CMD;
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

static int spi_flash_cs(SSISlave *dev, bool select)
{
	SPIFlashState *s = FROM_SSI_SLAVE(SPIFlashState, dev);
	dprintf("flash set cs %d\n", select);
	if(select)
		s->mode = SPI_FLASH_CMD;
	return 0;
}

static uint64_t spi_rom_read(void *opaque, hwaddr addr, unsigned size)
{
	SPIFlashState *s = opaque;

	switch(size)
	{
	case 4: return *(uint32_t *)(s->buf + addr);
	case 2: return *(uint16_t *)(s->buf + addr);
	case 1: return *(uint8_t *)(s->buf + addr);
	default: return *(uint64_t *)(s->buf + addr);
	}

}

static void spi_rom_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
	SPIFlashState *s = opaque;

	if(s->isram)
	{
		switch(size)
		{
			case 4: *(uint32_t *)(s->buf + addr) = data; break;
			case 2: *(uint16_t *)(s->buf + addr) = data; break;
			case 1: *(uint8_t *)(s->buf + addr) = data; break;
			default: *(uint64_t *)(s->buf + addr) = data; break;
		}
	}
}

static const MemoryRegionOps spi_rom_ops = {
    .read = spi_rom_read,
    .write = spi_rom_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static Property spi_flash_properties[] = {
    DEFINE_PROP_UINT32("size", SPIFlashState, size, 0),
    DEFINE_PROP_UINT64("addr", SPIFlashState, addr, 0),
    DEFINE_PROP_DRIVE("drive", SPIFlashState, blk),
    DEFINE_PROP_INT32("isram", SPIFlashState, isram, 0),
    DEFINE_PROP_END_OF_LIST(),
};



static void spi_flash_realize(SSISlave *dev, Error **errp)
{
	SPIFlashState *s = FROM_SSI_SLAVE(SPIFlashState, dev);
	int len;
	Error *local_err = NULL;

	s->mode = SPI_FLASH_CMD;
	memory_region_init_rom_device_nomigrate(&s->mem, OBJECT(dev), &spi_rom_ops , s, "spi flash rom", 0x800000, &local_err);
  	s->buf = memory_region_get_ram_ptr(&s->mem);

        memory_region_init_alias(&s->mem1, OBJECT(dev), "spi bios", &s->mem, 0, s->size);

	if(s->blk)
	{
		blk_set_perm(s->blk, BLK_PERM_ALL, BLK_PERM_ALL, errp);
		len = blk_getlength(s->blk);
		if(len != 0x800000) {
			printf("length=%d must be 8 mega bytes,run command bellow to trucate file to size:\n", len);
		blk_pwrite(s->blk, 0x800000-1, s->buf, 1, 0);
		}
		len = MIN(0x800000,len);
		blk_pread(s->blk, (int64_t)0LL, s->buf, len);
	}


	memory_region_add_subregion(get_system_memory(), s->addr, &s->mem1);
}

static void spi_flash_class_init(ObjectClass *klass, void *data)
{
    SSISlaveClass *k = SSI_SLAVE_CLASS(klass);
    DeviceClass *dc = DEVICE_CLASS(klass);

    k->realize = spi_flash_realize;
    k->transfer = spi_flash_transfer;
    k->set_cs = spi_flash_cs;
    k->cs_polarity = SSI_CS_LOW;
    dc->props = spi_flash_properties;
}


static const TypeInfo spi_flash_info = {
    .name          = "spi-flash",
    .parent        = TYPE_SSI_SLAVE,
    .instance_size = sizeof(SPIFlashState),
    .class_init    = spi_flash_class_init,
};

static void spi_flash_register_types(void)
{
    type_register_static(&spi_flash_info);
}

type_init(spi_flash_register_types)

