#include "qemu/osdep.h"
#include "hw/hw.h"
#include "sysemu/block-backend.h"
#include "hw/qdev.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "sysemu/blockdev.h"
#define DPRINTF(a...) //printf(a)

# include "hw/hw.h"
//# include "flash.h"
#define NAND_MFR_SAMSUNG	0xec
typedef struct NANDFlashState NANDFlashState;
/* FIXME: Pass block device as an argument.  */
#include "sysemu/sysemu.h"
#include "sysemu/dma.h"

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

# define NAND_IOSTATUS_ERROR	(1 << 0)
# define NAND_IOSTATUS_PLANE0	(1 << 1)
# define NAND_IOSTATUS_PLANE1	(1 << 2)
# define NAND_IOSTATUS_PLANE2	(1 << 3)
# define NAND_IOSTATUS_PLANE3	(1 << 4)
# define NAND_IOSTATUS_BUSY	(1 << 6)
# define NAND_IOSTATUS_UNPROTCT	(1 << 7)

# define MAX_PAGE		0x800
# define MAX_OOB		0x40

struct NANDFlashState {
    uint8_t manf_id, chip_id;
    int size, pages;
    int page_shift, oob_shift, erase_shift;
    uint8_t *storage;
    BlockBackend *blk;
    int mem_oob;

    int cle, ale, ce, wp, gnd;

    uint8_t io[MAX_PAGE + MAX_OOB + 0x400];
    uint8_t *ioaddr;
    int iolen;

    uint32_t cmd, addr;
    int addrlen;
    int status;
    int offset;

    void (*blk_write)(NANDFlashState *s);
    void (*blk_erase)(NANDFlashState *s);
    void (*blk_load)(NANDFlashState *s, uint32_t addr, int offset);
};

static void mem_and(uint8_t *dest, const uint8_t *src, size_t n)
{
    /* Like memcpy() but we logical-AND the data into the destination */
    int i;
    for (i = 0; i < n; i++) {
        dest[i] &= src[i];
    }
}
# define NAND_NO_AUTOINCR	0x00000001
# define NAND_BUSWIDTH_16	0x00000002
# define NAND_NO_PADDING	0x00000004
# define NAND_CACHEPRG		0x00000008
# define NAND_COPYBACK		0x00000010
# define NAND_IS_AND		0x00000020
# define NAND_4PAGE_ARRAY	0x00000040
# define NAND_NO_READRDY	0x00000100
# define NAND_SAMSUNG_LP	(NAND_NO_PADDING | NAND_COPYBACK)

# define NAND_IO
# define PAGE_SIZE		2048
# define PAGE_SHIFT		11
# define PAGE_SECTORS		4


# define PAGE(addr)		((addr) >> PAGE_SHIFT)
# define PAGE_START(page)	(PAGE(page) * (PAGE_SIZE + OOB_SIZE))
# define PAGE_MASK		((1 << PAGE_SHIFT) - 1)
# define OOB_SHIFT		(PAGE_SHIFT - 5)
# define OOB_SIZE		(1 << OOB_SHIFT)
# define SECTOR(addr)		((addr) >> (9))
# define SECTOR_OFFSET(addr)	((addr) & ((511 >> PAGE_SHIFT) << 8))


/* Program a single page */
static void glue(nand_blk_write_, PAGE_SIZE)(NANDFlashState *s)
{
    uint32_t off, page, sector, soff;
    uint8_t iobuf[(PAGE_SECTORS + 2) * 0x200];
    if (PAGE(s->addr) >= s->pages)
        return;

    if (!s->blk) {
        mem_and(s->storage + PAGE_START(s->addr) + (s->addr & PAGE_MASK) +
                        s->offset, s->io, s->iolen);
    } else if (s->mem_oob) {
        sector = SECTOR(s->addr);
        off = (s->addr & PAGE_MASK) + s->offset;
        soff = SECTOR_OFFSET(s->addr);
        if (blk_pread(s->blk, sector<<BDRV_SECTOR_BITS, iobuf, PAGE_SECTORS<<BDRV_SECTOR_BITS) == -1) {
            printf("%s: read error in sector %i\n", __FUNCTION__, sector);
            return;
        }

	DPRINTF("sector=%x,off=%x,soff=%x,s->iolen=%x\n",sector,off,soff,s->iolen);

        mem_and(iobuf + (soff | off), s->io, MIN(s->iolen, PAGE_SIZE - off));
        if (off + s->iolen > PAGE_SIZE) {
            page = PAGE(s->addr);
            mem_and(s->storage + (page << OOB_SHIFT), s->io + PAGE_SIZE - off,
                            MIN(OOB_SIZE, off + s->iolen - PAGE_SIZE));
        }

        if (blk_pwrite(s->blk, sector<<BDRV_SECTOR_BITS, iobuf, PAGE_SECTORS<<BDRV_SECTOR_BITS,0) == -1)
            printf("%s: write error in sector %i\n", __FUNCTION__, sector);
    } else {
        off = PAGE_START(s->addr) + (s->addr & PAGE_MASK) + s->offset;
        sector = off >> 9;
        soff = off & 0x1ff;
        if (blk_pread(s->blk, sector<<BDRV_SECTOR_BITS, iobuf, (PAGE_SECTORS + 2)<<BDRV_SECTOR_BITS) == -1) {
            printf("%s: read error in sector %i\n", __FUNCTION__, sector);
            return;
        }

        memcpy(iobuf + soff, s->io, s->iolen);

        if (blk_pwrite(s->blk, sector<<BDRV_SECTOR_BITS, iobuf, (PAGE_SECTORS + 2)<<BDRV_SECTOR_BITS,0) == -1)
            printf("%s: write error in sector %i\n", __FUNCTION__, sector);
    }
    s->offset = 0;
}

/* Erase a single block */
static void glue(nand_blk_erase_, PAGE_SIZE)(NANDFlashState *s)
{
    uint32_t i, page, addr;
    uint8_t iobuf[0x200] = { [0 ... 0x1ff] = 0xff, };
    addr = s->addr & ~((1 << (s->page_shift + s->erase_shift)) - 1);

    if (PAGE(addr) >= s->pages)
        return;

    if (!s->blk) {
        memset(s->storage + PAGE_START(addr),
                        0xff, (PAGE_SIZE + OOB_SIZE) << s->erase_shift);
    } else if (s->mem_oob) {
        memset(s->storage + (PAGE(addr) << OOB_SHIFT),
                        0xff, OOB_SIZE << s->erase_shift);
        i = SECTOR(addr);
        page = SECTOR(addr + (1<<(s->page_shift + s->erase_shift)));
        for (; i < page; i ++)
            if (blk_pwrite(s->blk, i<<BDRV_SECTOR_BITS, iobuf, 1<<BDRV_SECTOR_BITS,0) == -1)
                printf("%s: write error in sector %i\n", __FUNCTION__, i);
    } else {
        addr = PAGE_START(addr);
        page = addr >> 9;
        if (blk_pread(s->blk, page<<BDRV_SECTOR_BITS, iobuf, 1<<BDRV_SECTOR_BITS) == -1)
            printf("%s: read error in sector %i\n", __FUNCTION__, page);
        memset(iobuf + (addr & 0x1ff), 0xff, (~addr & 0x1ff) + 1);
        if (blk_pwrite(s->blk, page<<BDRV_SECTOR_BITS, iobuf, 1<<BDRV_SECTOR_BITS,0) == -1)
            printf("%s: write error in sector %i\n", __FUNCTION__, page);

        memset(iobuf, 0xff, 0x200);
        i = (addr & ~0x1ff) + 0x200;
        for (addr += ((PAGE_SIZE + OOB_SIZE) << s->erase_shift) - 0x200;
                        i < addr; i += 0x200)
            if (blk_pwrite(s->blk, i >> 9<<BDRV_SECTOR_BITS, iobuf, 1<<BDRV_SECTOR_BITS,0) == -1)
                printf("%s: write error in sector %i\n", __FUNCTION__, i >> 9);

        page = i >> 9;
        if (blk_pread(s->blk, page<<BDRV_SECTOR_BITS, iobuf, 1<<BDRV_SECTOR_BITS) == -1)
            printf("%s: read error in sector %i\n", __FUNCTION__, page);
        memset(iobuf, 0xff, ((addr - 1) & 0x1ff) + 1);
        if (blk_pwrite(s->blk, page<<BDRV_SECTOR_BITS, iobuf, 1<<BDRV_SECTOR_BITS,0) == -1)
            printf("%s: write error in sector %i\n", __FUNCTION__, page);
    }
}

static void glue(nand_blk_load_, PAGE_SIZE)(NANDFlashState *s,
                uint32_t addr, int offset)
{
    if (PAGE(addr) >= s->pages)
        return;

    if (s->blk) {
        if (s->mem_oob) {
            if (blk_pread(s->blk, SECTOR(addr)<<BDRV_SECTOR_BITS, s->io, PAGE_SECTORS<<BDRV_SECTOR_BITS) == -1)
                printf("%s: read error in sector %i\n",
                                __FUNCTION__, SECTOR(addr));
            memcpy(s->io + SECTOR_OFFSET(s->addr) + PAGE_SIZE,
                            s->storage + (PAGE(s->addr) << OOB_SHIFT),
                            OOB_SIZE);
            s->ioaddr = s->io + SECTOR_OFFSET(s->addr) + offset;
        } else {
            if (blk_pread(s->blk, PAGE_START(addr),
                                    s->io,( PAGE_SECTORS + 2)<<BDRV_SECTOR_BITS) == -1)
                printf("%s: read error in sector %i\n",
                                __FUNCTION__, PAGE_START(addr) >> 9);
            s->ioaddr = s->io + (PAGE_START(addr) & 0x1ff) + offset;
        }
    } else {
        memcpy(s->io, s->storage + PAGE_START(s->addr) +
                        offset, PAGE_SIZE + OOB_SIZE - offset);
        s->ioaddr = s->io;
    }

    s->addr &= PAGE_SIZE - 1;
    s->addr += PAGE_SIZE;
}

static void glue(nand_init_, PAGE_SIZE)(NANDFlashState *s)
{
    s->oob_shift = PAGE_SHIFT - 5;
    s->pages = s->size >> PAGE_SHIFT;

    s->blk_erase = glue(nand_blk_erase_, PAGE_SIZE);
    s->blk_write = glue(nand_blk_write_, PAGE_SIZE);
    s->blk_load = glue(nand_blk_load_, PAGE_SIZE);
}

/* Information based on Linux drivers/mtd/nand/nand_ids.c */
static const struct {
    int size;
    int width;
    int page_shift;
    int erase_shift;
    uint32_t options;
} nand_flash_ids[0x100] = {
    [0 ... 0xff] = { 0 },

    [0x6e] = { 1,	8,	8, 4, 0 },
    [0x64] = { 2,	8,	8, 4, 0 },
    [0x6b] = { 4,	8,	9, 4, 0 },
    [0xe8] = { 1,	8,	8, 4, 0 },
    [0xec] = { 1,	8,	8, 4, 0 },
    [0xea] = { 2,	8,	8, 4, 0 },
    [0xd5] = { 4,	8,	9, 4, 0 },
    [0xe3] = { 4,	8,	9, 4, 0 },
    [0xe5] = { 4,	8,	9, 4, 0 },
    [0xd6] = { 8,	8,	9, 4, 0 },

    [0x39] = { 8,	8,	9, 4, 0 },
    [0xe6] = { 8,	8,	9, 4, 0 },
    [0x49] = { 8,	16,	9, 4, NAND_BUSWIDTH_16 },
    [0x59] = { 8,	16,	9, 4, NAND_BUSWIDTH_16 },

    [0x33] = { 16,	8,	9, 5, 0 },
    [0x73] = { 16,	8,	9, 5, 0 },
    [0x43] = { 16,	16,	9, 5, NAND_BUSWIDTH_16 },
    [0x53] = { 16,	16,	9, 5, NAND_BUSWIDTH_16 },

    [0x35] = { 32,	8,	9, 5, 0 },
    [0x75] = { 32,	8,	9, 5, 0 },
    [0x45] = { 32,	16,	9, 5, NAND_BUSWIDTH_16 },
    [0x55] = { 32,	16,	9, 5, NAND_BUSWIDTH_16 },

    [0x36] = { 64,	8,	9, 5, 0 },
    [0x76] = { 64,	8,	9, 5, 0 },
    [0x46] = { 64,	16,	9, 5, NAND_BUSWIDTH_16 },
    [0x56] = { 64,	16,	9, 5, NAND_BUSWIDTH_16 },

    [0x78] = { 128,	8,	9, 5, 0 },
    [0x39] = { 128,	8,	9, 5, 0 },
    [0x79] = { 128,	8,	9, 5, 0 },
    [0x72] = { 128,	16,	9, 5, NAND_BUSWIDTH_16 },
    [0x49] = { 128,	16,	9, 5, NAND_BUSWIDTH_16 },
    [0x74] = { 128,	16,	9, 5, NAND_BUSWIDTH_16 },
    [0x59] = { 128,	16,	9, 5, NAND_BUSWIDTH_16 },

    [0x71] = { 256,	8,	9, 5, 0 },

    /*
     * These are the new chips with large page size. The pagesize and the
     * erasesize is determined from the extended id bytes
     */
# define LP_OPTIONS	(NAND_SAMSUNG_LP | NAND_NO_READRDY | NAND_NO_AUTOINCR)
# define LP_OPTIONS16	(LP_OPTIONS | NAND_BUSWIDTH_16)

    /* 512 Megabit */
    [0xa2] = { 64,	8,	0, 0, LP_OPTIONS },
    [0xf2] = { 64,	8,	0, 0, LP_OPTIONS },
    [0xb2] = { 64,	16,	0, 0, LP_OPTIONS16 },
    [0xc2] = { 64,	16,	0, 0, LP_OPTIONS16 },

    /* 1 Gigabit */
    [0xa1] = { 128,	8,	0, 0, LP_OPTIONS },
    [0xf1] = { 128,	8,	0, 0, LP_OPTIONS },
    [0xb1] = { 128,	16,	0, 0, LP_OPTIONS16 },
    [0xc1] = { 128,	16,	0, 0, LP_OPTIONS16 },

    /* 2 Gigabit */
    [0xaa] = { 256,	8,	0, 0, LP_OPTIONS },
    [0xda] = { 256,	8,	0, 0, LP_OPTIONS },
    [0xba] = { 256,	16,	0, 0, LP_OPTIONS16 },
    [0xca] = { 256,	16,	0, 0, LP_OPTIONS16 },

    /* 4 Gigabit */
    [0xac] = { 512,	8,	0, 0, LP_OPTIONS },
    [0xdc] = { 512,	8,	0, 0, LP_OPTIONS },
    [0xbc] = { 512,	16,	0, 0, LP_OPTIONS16 },
    [0xcc] = { 512,	16,	0, 0, LP_OPTIONS16 },

    /* 8 Gigabit */
    [0xa3] = { 1024,	8,	0, 0, LP_OPTIONS },
    [0xd3] = { 1024,	8,	0, 0, LP_OPTIONS },
    [0xb3] = { 1024,	16,	0, 0, LP_OPTIONS16 },
    [0xc3] = { 1024,	16,	0, 0, LP_OPTIONS16 },

    /* 16 Gigabit */
    [0xa5] = { 2048,	8,	0, 0, LP_OPTIONS },
    [0xd5] = { 2048,	8,	0, 0, LP_OPTIONS },
    [0xb5] = { 2048,	16,	0, 0, LP_OPTIONS16 },
    [0xc5] = { 2048,	16,	0, 0, LP_OPTIONS16 },
};

static void nand_reset(NANDFlashState *s)
{
    s->cmd = NAND_CMD_READ0;
    s->addr = 0;
    s->addrlen = 0;
    s->iolen = 0;
    s->offset = 0;
    s->status &= NAND_IOSTATUS_UNPROTCT;
}

static void nand_command(NANDFlashState *s)
{
    switch (s->cmd) {
    case NAND_CMD_READ0:
        s->iolen = 0;
        break;

    case NAND_CMD_READID:
        s->io[0] = s->manf_id;
        s->io[1] = s->chip_id;
        s->io[2] = 'Q';		/* Don't-care byte (often 0xa5) */
        if (nand_flash_ids[s->chip_id].options & NAND_SAMSUNG_LP)
            s->io[3] = 0x15;	/* Page Size, Block Size, Spare Size.. */
        else
            s->io[3] = 0xc0;	/* Multi-plane */
        s->ioaddr = s->io;
        s->iolen = 4;
        break;

    case NAND_CMD_RANDOMREAD2:
    case NAND_CMD_NOSERIALREAD2:
        if (!(nand_flash_ids[s->chip_id].options & NAND_SAMSUNG_LP))
            break;

        s->blk_load(s, s->addr, s->addr & ((1 << s->page_shift) - 1));
        break;

    case NAND_CMD_RESET:
        nand_reset(s);
        break;

    case NAND_CMD_PAGEPROGRAM1:
        s->ioaddr = s->io;
        s->iolen = 0;
        break;

    case NAND_CMD_PAGEPROGRAM2:
        if (s->wp) {
            s->blk_write(s);
        }
        break;

    case NAND_CMD_BLOCKERASE1:
        break;

    case NAND_CMD_BLOCKERASE2:
        if (nand_flash_ids[s->chip_id].options & NAND_SAMSUNG_LP)
            s->addr <<= 16;
        else
            s->addr <<= 8;

        if (s->wp) {
            s->blk_erase(s);
        }
        break;

    case NAND_CMD_READSTATUS:
        s->io[0] = s->status;
        s->ioaddr = s->io;
        s->iolen = 1;
        break;

    default:
        printf("%s: Unknown NAND command 0x%02x\n", __FUNCTION__, s->cmd);
    }
}


static NANDFlashState *nand_init(int manf_id, int chip_id)
{
    int pagesize;
    NANDFlashState *s;
    DriveInfo *nand;

    if (nand_flash_ids[chip_id].size == 0) {
        hw_error("%s: Unsupported NAND chip ID.\n", __FUNCTION__);
    }

    s = (NANDFlashState *) g_malloc0(sizeof(NANDFlashState));
    nand = drive_get_next(IF_MTD);
    if (nand)
        s->blk = blk_by_legacy_dinfo(nand);
    s->manf_id = manf_id;
    s->chip_id = chip_id;
    s->size = nand_flash_ids[s->chip_id].size << 20;
    if (nand_flash_ids[s->chip_id].options & NAND_SAMSUNG_LP) {
        s->page_shift = 11;
        s->erase_shift = 6;
    } else {
        s->page_shift = nand_flash_ids[s->chip_id].page_shift;
        s->erase_shift = nand_flash_ids[s->chip_id].erase_shift;
    }

    switch (1 << s->page_shift) {
    case 2048:
        nand_init_2048(s);
        break;
    default:
        hw_error("%s: Unsupported NAND block size.\n", __FUNCTION__);
    }

    pagesize = 1 << s->oob_shift;
    s->mem_oob = 1;
    if (s->blk && blk_getlength(s->blk) >=
                    (s->pages << s->page_shift) + (s->pages << s->oob_shift)) {
        pagesize = 0;
        s->mem_oob = 0;
    }

    if (!s->blk)
	{
        pagesize += 1 << s->page_shift;
        s->mem_oob = 0;
	}

    if (pagesize)
        s->storage = (uint8_t *) memset(g_malloc(s->pages * pagesize),
                        0xff, s->pages * pagesize);
    /* Give s->ioaddr a sane value in case we save state before it
       is used.  */
    s->ioaddr = s->io;


    return s;
}



# undef PAGE_SIZE
# undef PAGE_SHIFT
# undef PAGE_SECTORS
# undef ADDR_SHIFT
/*
trick for link ok
*/

/*
 * ls1a nand begin
 *
 */
/*
 *
 * 	only support nand:
	read write 2k
	erase 128k
	that is chips in nand_flash_ids whith NAND_SAMSUNG_LP set,nand_command_lp for linux.
	large page nandflash.
        page_shift = 11;
        erase_shift = 6;
	oob_shit = 6, oob size 0x40

	address is same with op with nand flash directly except erase command.
	erase command's address here is same as read/write address format,does not shift page_shit.

	for large page device no seperate oob,oob is after writedata.
        NAND_CMD_READ_OOB command to read OOB,because same address with NAND_CMD_READ0(NAND_CMD_READSTART for lp).

linux:
	nand_command(struct mtd_info *mtd, unsigned int command, int column, int page_addr);
	 nand address = page_addr << PAGE_SHIFT + column;
	 for oob offset is begin form 0
        nand_command_lp
	  nand address = page_addr << PAGE_SHIFT + column;
	           for oob offset is begin form write_size

 *
 * nand_command_lp - [DEFAULT] Send command to NAND large page device
 * @mtd:    MTD device structure
 * @command:    the command to be sent
 * @column: the column address for this command, -1 if none
 * @page_addr:  the page address for this command, -1 if none
 *
 * Send command to NAND device. This is the version for the new large page
 * devices We dont have the separate regions as we have in the small page
 * devices.  We must emulate NAND_CMD_READOOB to keep the code compatible.
static void nand_command_lp(struct mtd_info *mtd, unsigned int command,
                int column, int page_addr)
{
    register struct nand_chip *chip = mtd->priv;

    // Emulate NAND_CMD_READOOB 
    if (command == NAND_CMD_READOOB) {
        column += mtd->writesize;
        command = NAND_CMD_READ0;
    }
 
...

}

 * nand_command - [DEFAULT] Send command to NAND device
 * @mtd:	MTD device structure
 * @command:	the command to be sent
 * @column:	the column address for this command, -1 if none
 * @page_addr:	the page address for this command, -1 if none
 *
 * Send command to NAND device. This function is used for small page
 * devices (256/512 Bytes per page)
/
static void nand_command(struct mtd_info *mtd, unsigned int command,
			 int column, int page_addr)
{
	register struct nand_chip *chip = mtd->priv;
	int ctrl = NAND_CTRL_CLE | NAND_CTRL_CHANGE;

	
	 * Write out the command to the device.
	/
	if (command == NAND_CMD_SEQIN) {
		int readcmd;

		if (column >= mtd->writesize) {
			* OOB area *
			column -= mtd->writesize;
			readcmd = NAND_CMD_READOOB;
		} else if (column < 256) {
			* First 256 bytes --> READ0 *
			readcmd = NAND_CMD_READ0;
		} else {
			column -= 256;
			readcmd = NAND_CMD_READ1;
		}
		chip->cmd_ctrl(mtd, readcmd, ctrl);
		ctrl &= ~NAND_CTRL_CHANGE;
	}
	chip->cmd_ctrl(mtd, command, ctrl);

 */
#define PAGE_SHIFT 11
#define PAGE_SIZE (1 << PAGE_SHIFT)
#define OOB_SHIFT (PAGE_SHIFT - 5)
#define OOB_SIZE (1 << OOB_SHIFT)
#define ERASE_SHIFT 6
#define ERASE_SIZE (1 << (ERASE_SHIFT + PAGE_SHIFT))

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


#define ADDR_SPARE(s) ((s->regs.addr_l&0xfff)|((long long)s->regs.addr_h<<16))
#define ADDR_NOSPARE(s) ((s->regs.addr_l&0xfff)|((long long)s->regs.addr_h<<16))

static int nand_load_next(NandState *s)
{
	uint32_t tmp,offset,addr,iolen;
	int cmd;
	cmd = s->regs.cmd;

	/*nand addr maybe not page aligned*/
	if( (cmd & (CMD_SPARE |CMD_MAIN)) == CMD_SPARE)
	{
		tmp = ADDR_SPARE(s);
		addr = (tmp >> 16) << PAGE_SHIFT;
		offset =  tmp & 0xffff;
		iolen = (1 << s->chip->oob_shift) - offset;
	}
	else if((cmd & (CMD_SPARE |CMD_MAIN)) == (CMD_SPARE|CMD_MAIN))
	{
		tmp = ADDR_SPARE(s);
		addr = (tmp >> 16) << PAGE_SHIFT;
		offset =  tmp & 0xffff;
		iolen = (1 << s->chip->page_shift) + (1 << s->chip->oob_shift) - offset;
	}
	else
	{
		tmp = ADDR_NOSPARE(s);
		addr = (tmp >> 16) << PAGE_SHIFT;
		offset =  tmp & (PAGE_SIZE -1);
		iolen = (1 << s->chip->page_shift) - offset;
	}

	s->chip->addr = addr;
	s->chip->blk_load(s->chip, addr, offset);
	s->chip->iolen = iolen;
	s->regs.addr_h += 0x1;
	return iolen;
}


static int nand_write_next(NandState *s)
{
	uint32_t offset,addr,iolen;
	uint64_t tmp;
	int cmd;
	cmd = s->regs.cmd;

again:

	/*nand addr maybe not page aligned*/
	if( (cmd & (CMD_SPARE |CMD_MAIN)) == CMD_SPARE)
	{
/*addr include 3 row addr and 2 col addr
 * one sectior size == pagesize + oobsize,
 */
		tmp = ADDR_SPARE(s);
		addr = (tmp >> 16) << PAGE_SHIFT;
		offset =  tmp & (PAGE_SIZE -1);
		iolen = (1 << s->chip->oob_shift) - offset;
	}
	else if((cmd & (CMD_SPARE |CMD_MAIN)) == (CMD_SPARE|CMD_MAIN))
	{
		tmp = ADDR_SPARE(s);
		addr = (tmp >> 16) << PAGE_SHIFT;
		offset =  (tmp & 0xffff);
		iolen = (1 << s->chip->page_shift) + (1 << s->chip->oob_shift) - offset;
	}
	else
	{
		tmp = ADDR_NOSPARE(s);
		addr = (tmp >> 16) << PAGE_SHIFT;
		offset =  tmp & (PAGE_SIZE -1);
		iolen = (1 << s->chip->page_shift) - offset;
	}

	if(s->chip->iolen >= iolen)
	{
		s->chip->addr = addr;
		s->chip->offset = offset;
		s->chip->iolen = iolen;

		s->chip->blk_write(s->chip);

		s->chip->ioaddr = s->chip->io;
		s->chip->iolen = 0;
		s->regs.addr_h += 0x1;
		goto again;
	}

 return iolen - s->chip->iolen;
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
		s->regs.op_num -= copied;

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
		s->regs.cmd |= CMD_DONE;
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

			for(i=0,addr = (ADDR_NOSPARE(s)>>16)<<PAGE_SHIFT; i < s->regs.op_num; i++,addr += 64<<PAGE_SHIFT)
			{
				s->chip->addr = addr;
				s->chip->blk_erase(s->chip);
			}

		}
		else
		{
			s->chip->addr = (ADDR_NOSPARE(s)>>16)<<PAGE_SHIFT ;
			s->chip->blk_erase(s->chip);
		}

		s->regs.cmd |= CMD_DONE;
		s->regs.cmd &= ~CMD_VALID;
	}
	else if(cmd & CMD_READID)
	{
		s->chip->cmd = NAND_CMD_READID;
		nand_command(s->chip);
		memcpy(&s->regs.id_l,s->chip->io,4);
		s->regs.id_l = 0xff|(s->chip->io[1]<<24)|(s->chip->io[2]<<16)|(s->chip->io[3]<<8);
		s->regs.status_id_h = s->chip->io[0];
		s->regs.cmd |= CMD_DONE;
		s->regs.cmd &= ~CMD_VALID;
	}
	else if(cmd & NAND_CMD_READSTATUS)
	{
		s->chip->cmd = NAND_CMD_READSTATUS;
		nand_command(s->chip);
		s->regs.status_id_h &= 0xff;
		memcpy((char *)(&s->regs.status_id_h)+1,s->chip->io,1);
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
    nand_sysbus_state *d = SYS_BUS_LS1ANAND(dev);
    //memset(&d->nand,0,sizeof(d->nand));
    d->nand.chip = nand_init(NAND_MFR_SAMSUNG, 0xa1);
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
    DEFINE_PROP_DRIVE("drive", NANDFlashState, blk),
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
