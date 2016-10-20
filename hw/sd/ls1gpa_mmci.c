/*
 * Loongson LS1GPA MultiMediaCard/SD/SDIO Controller emulation.
 *
 * Copyright (c) 2006 Openedhand Ltd.
 * Written by Andrzej Zaborowski <balrog@zabor.org>
 *
 * This code is licensed under the GPLv2.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/sd/sd.h"
#include "hw/qdev.h"
#include "hw/qdev-properties.h"
#include "qemu/error-report.h"

# define REG_FMT		"0x" TARGET_FMT_plx
typedef struct LS1GPA_MMCIState LS1GPA_MMCIState;
LS1GPA_MMCIState *ls1gpa_mmci_init(MemoryRegion *sysmem,
                hwaddr base,
                BlockBackend *blk, qemu_irq irq,
                qemu_irq rx_dma, qemu_irq tx_dma);

void ls1gpa_mmci_handlers(LS1GPA_MMCIState *s, qemu_irq readonly,
                          qemu_irq coverswitch);

#define TYPE_LS1GPA_MMCI "ls1gpa-mmci"
#define LS1GPA_MMCI(obj) OBJECT_CHECK(LS1GPA_MMCIState, (obj), TYPE_LS1GPA_MMCI)

#define TYPE_LS1GPA_MMCI_BUS "ls1gpa-mmci-bus"
#define LS1GPA_MMCI_BUS(obj) OBJECT_CHECK(SDBus, (obj), TYPE_LS1GPA_MMCI_BUS)

#define SDICON     0x00
#define SDIPRE     0x04
#define SDICMDARG  0x08
#define SDICMDCON  0x0c
#define SDICMDSTA  0x10
#define SDIRSP0    0x14
#define SDIRSP1    0x18
#define SDIRSP2    0x1C
#define SDIRSP3    0x20
#define SDIDTIMER  0x24
#define SDIBSIZE   0x28
#define SDIDATCON  0x2C
//#define SDIDCON               (0x2C)
#define SDIDATCNT  0x30
#define SDIDSTA               (0x34)
#define SDIFSTA    0x38
#define SDIINTMSK  0x3C
#define SDIWRDAT   0x40
#define SDISTAADD0 0x44
#define SDISTAADD1 0x48
#define SDISTAADD2 0x4c
#define SDISTAADD3 0x50
#define SDISTAADD4 0x54
#define SDISTAADD5 0x58
#define SDISTAADD6 0x5c
#define SDISTAADD7 0x60
#define SDIINTEN   0x64

typedef struct ls1gpa_sdio_regs {
    int sdicon;
    int sdipre;
    int sdicmdarg;
    int sdicmdcon;
    int sdicmdsta;
    int sdirsp0;
    int sdirsp1;
    int sdirsp2;
    int sdirsp3;
    int sdidtimer;
    int sdibsize;
    int sdidatcon;
    int sdidatcnt;
    int sdidsta;
    int sdifsta;
    int sdiintmsk;
    int sdiwrdat;
    int sdistaadd0;
    int sdistaadd1;
    int sdistaadd2;
    int sdistaadd3;
    int sdistaadd4;
    int sdistaadd5;
    int sdistaadd6;
    int sdistaadd7;
    int sdiinten;
} ls1gpa_sdio_regs_t;

struct LS1GPA_MMCIState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq rx_dma;
    qemu_irq tx_dma;
    qemu_irq inserted;
    qemu_irq readonly;

    BlockBackend *blk;
    SDBus sdbus;

    uint32_t status;
    uint32_t clkrt;
    uint32_t spi;
    uint32_t cmdat;
    uint32_t resp_tout;
    uint32_t read_tout;
    int32_t blklen;
    int32_t numblk;
    uint32_t intmask;
    uint32_t intreq;
    int32_t cmd;
    uint32_t arg;

    int32_t active;
    int32_t bytesleft;
    uint8_t tx_fifo[64];
    uint32_t tx_start;
    uint32_t tx_len;
    uint8_t rx_fifo[32];
    uint32_t rx_start;
    uint32_t rx_len;
    uint16_t resp_fifo[9];
    uint32_t resp_len;

    int32_t cmdreq;

    union{
    ls1gpa_sdio_regs_t sdioregs;
    uint32_t regdata[0x60/4];
    };
};

static bool ls1gpa_mmci_vmstate_validate(void *opaque, int version_id)
{
    LS1GPA_MMCIState *s = opaque;

    return s->tx_start < ARRAY_SIZE(s->tx_fifo)
        && s->rx_start < ARRAY_SIZE(s->rx_fifo)
        && s->tx_len <= ARRAY_SIZE(s->tx_fifo)
        && s->rx_len <= ARRAY_SIZE(s->rx_fifo)
        && s->resp_len <= ARRAY_SIZE(s->resp_fifo);
}


static const VMStateDescription vmstate_ls1gpa_mmci = {
    .name = "ls1gpa-mmci",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(status, LS1GPA_MMCIState),
        VMSTATE_UINT32(clkrt, LS1GPA_MMCIState),
        VMSTATE_UINT32(spi, LS1GPA_MMCIState),
        VMSTATE_UINT32(cmdat, LS1GPA_MMCIState),
        VMSTATE_UINT32(resp_tout, LS1GPA_MMCIState),
        VMSTATE_UINT32(read_tout, LS1GPA_MMCIState),
        VMSTATE_INT32(blklen, LS1GPA_MMCIState),
        VMSTATE_INT32(numblk, LS1GPA_MMCIState),
        VMSTATE_UINT32(intmask, LS1GPA_MMCIState),
        VMSTATE_UINT32(intreq, LS1GPA_MMCIState),
        VMSTATE_INT32(cmd, LS1GPA_MMCIState),
        VMSTATE_UINT32(arg, LS1GPA_MMCIState),
        VMSTATE_INT32(cmdreq, LS1GPA_MMCIState),
        VMSTATE_INT32(active, LS1GPA_MMCIState),
        VMSTATE_INT32(bytesleft, LS1GPA_MMCIState),
        VMSTATE_UINT32(tx_start, LS1GPA_MMCIState),
        VMSTATE_UINT32(tx_len, LS1GPA_MMCIState),
        VMSTATE_UINT32(rx_start, LS1GPA_MMCIState),
        VMSTATE_UINT32(rx_len, LS1GPA_MMCIState),
        VMSTATE_UINT32(resp_len, LS1GPA_MMCIState),
        VMSTATE_VALIDATE("fifo size incorrect", ls1gpa_mmci_vmstate_validate),
        VMSTATE_UINT8_ARRAY(tx_fifo, LS1GPA_MMCIState, 64),
        VMSTATE_UINT8_ARRAY(rx_fifo, LS1GPA_MMCIState, 32),
        VMSTATE_UINT16_ARRAY(resp_fifo, LS1GPA_MMCIState, 9),
        VMSTATE_END_OF_LIST()
    }
};

#define MMC_STRPCL	0x00	/* MMC Clock Start/Stop register */
#define MMC_STAT	0x04	/* MMC Status register */
#define MMC_CLKRT	0x08	/* MMC Clock Rate register */
#define MMC_SPI		0x0c	/* MMC SPI Mode register */
#define MMC_CMDAT	0x10	/* MMC Command/Data register */
#define MMC_RESTO	0x14	/* MMC Response Time-Out register */
#define MMC_RDTO	0x18	/* MMC Read Time-Out register */
#define MMC_BLKLEN	0x1c	/* MMC Block Length register */
#define MMC_NUMBLK	0x20	/* MMC Number of Blocks register */
#define MMC_PRTBUF	0x24	/* MMC Buffer Partly Full register */
#define MMC_I_MASK	0x28	/* MMC Interrupt Mask register */
#define MMC_I_REG	0x2c	/* MMC Interrupt Request register */
#define MMC_CMD		0x30	/* MMC Command register */
#define MMC_ARGH	0x34	/* MMC Argument High register */
#define MMC_ARGL	0x38	/* MMC Argument Low register */
#define MMC_RES		0x3c	/* MMC Response FIFO */
#define MMC_RXFIFO	0x40	/* MMC Receive FIFO */
#define MMC_TXFIFO	0x44	/* MMC Transmit FIFO */
#define MMC_RDWAIT	0x48	/* MMC RD_WAIT register */
#define MMC_BLKS_REM	0x4c	/* MMC Blocks Remaining register */

/* Bitfield masks */
#define STRPCL_STOP_CLK	(1 << 0)
#define STRPCL_STRT_CLK	(1 << 1)
#define STAT_TOUT_RES	(1 << 1)
#define STAT_CLK_EN	(1 << 8)
#define STAT_DATA_DONE	(1 << 11)
#define STAT_PRG_DONE	(1 << 12)
#define STAT_END_CMDRES	(1 << 13)
#define SPI_SPI_MODE	(1 << 0)
#define CMDAT_RES_TYPE	(3 << 0)
#define CMDAT_DATA_EN	(1 << 2)
#define CMDAT_WR_RD	(1 << 3)
#define CMDAT_DMA_EN	(1 << 7)
#define CMDAT_STOP_TRAN	(1 << 10)
#define INT_DATA_DONE	(1 << 0)
#define INT_PRG_DONE	(1 << 1)
#define INT_END_CMD	(1 << 2)
#define INT_STOP_CMD	(1 << 3)
#define INT_CLK_OFF	(1 << 4)
#define INT_RXFIFO_REQ	(1 << 5)
#define INT_TXFIFO_REQ	(1 << 6)
#define INT_TINT	(1 << 7)
#define INT_DAT_ERR	(1 << 8)
#define INT_RES_ERR	(1 << 9)
#define INT_RD_STALLED	(1 << 10)
#define INT_SDIO_INT	(1 << 11)
#define INT_SDIO_SACK	(1 << 12)
#define PRTBUF_PRT_BUF	(1 << 0)

/* Route internal interrupt lines to the global IC and DMA */
static void ls1gpa_mmci_int_update(LS1GPA_MMCIState *s)
{
    uint32_t mask = s->intmask;
    if (s->cmdat & CMDAT_DMA_EN) {
        mask |= INT_RXFIFO_REQ | INT_TXFIFO_REQ;

        qemu_set_irq(s->rx_dma, !!(s->intreq & INT_RXFIFO_REQ));
        qemu_set_irq(s->tx_dma, !!(s->intreq & INT_TXFIFO_REQ));
    }

    qemu_set_irq(s->irq, !!(s->intreq & ~mask));
}

static void ls1gpa_mmci_fifo_update(LS1GPA_MMCIState *s)
{
    if (!s->active)
        return;

    if (s->cmdat & CMDAT_WR_RD) {
        while (s->bytesleft && s->tx_len) {
            sdbus_write_data(&s->sdbus, s->tx_fifo[s->tx_start++]);
            s->tx_start &= 0x1f;
            s->tx_len --;
            s->bytesleft --;
        }
        if (s->bytesleft)
            s->intreq |= INT_TXFIFO_REQ;
    } else
        while (s->bytesleft && s->rx_len < 32) {
            s->rx_fifo[(s->rx_start + (s->rx_len ++)) & 0x1f] =
                sdbus_read_data(&s->sdbus);
            s->bytesleft --;
            s->intreq |= INT_RXFIFO_REQ;
        }

    if (!s->bytesleft) {
        s->active = 0;
        s->intreq |= INT_DATA_DONE;
        s->status |= STAT_DATA_DONE;

        if (s->cmdat & CMDAT_WR_RD) {
            s->intreq |= INT_PRG_DONE;
            s->status |= STAT_PRG_DONE;
        }
    }

    ls1gpa_mmci_int_update(s);
}

static void ls1gpa_mmci_wakequeues(LS1GPA_MMCIState *s)
{
    int rsplen, i;
    SDRequest request;
    uint8_t response[16];

    s->active = 1;
    s->rx_len = 0;
    s->tx_len = 0;
    s->cmdreq = 0;

    request.cmd = s->cmd;
    request.arg = s->arg;
    request.crc = 0;	/* FIXME */

    rsplen = sdbus_do_command(&s->sdbus, &request, response);
    s->intreq |= INT_END_CMD;

    memset(s->resp_fifo, 0, sizeof(s->resp_fifo));
    switch (s->cmdat & CMDAT_RES_TYPE) {
#define LS1GPA_MMCI_RESP(wd, value0, value1)	\
        s->resp_fifo[(wd) + 0] |= (value0);	\
        s->resp_fifo[(wd) + 1] |= (value1) << 8;
    case 0:	/* No response */
        goto complete;

    case 1:	/* R1, R4, R5 or R6 */
        if (rsplen < 4)
            goto timeout;
        goto complete;

    case 2:	/* R2 */
        if (rsplen < 16)
            goto timeout;
        goto complete;

    case 3:	/* R3 */
        if (rsplen < 4)
            goto timeout;
        goto complete;

    complete:
        for (i = 0; rsplen > 0; i ++, rsplen -= 2) {
            LS1GPA_MMCI_RESP(i, response[i * 2], response[i * 2 + 1]);
        }
        s->status |= STAT_END_CMDRES;

        if (!(s->cmdat & CMDAT_DATA_EN))
            s->active = 0;
        else
            s->bytesleft = s->numblk * s->blklen;

        s->resp_len = 0;
        break;

    timeout:
        s->active = 0;
        s->status |= STAT_TOUT_RES;
        break;
    }

    ls1gpa_mmci_fifo_update(s);
}

static uint64_t ls1gpa_mmci_read(void *opaque, hwaddr offset, unsigned size)
{
    LS1GPA_MMCIState *s = (LS1GPA_MMCIState *) opaque;
    uint32_t ret;

    switch (offset) {
    case MMC_STRPCL:
        return 0;
    case MMC_STAT:
        return s->status;
    case MMC_CLKRT:
        return s->clkrt;
    case MMC_SPI:
        return s->spi;
    case MMC_CMDAT:
        return s->cmdat;
    case MMC_RESTO:
        return s->resp_tout;
    case MMC_RDTO:
        return s->read_tout;
    case MMC_BLKLEN:
        return s->blklen;
    case MMC_NUMBLK:
        return s->numblk;
    case MMC_PRTBUF:
        return 0;
    case MMC_I_MASK:
        return s->intmask;
    case MMC_I_REG:
        return s->intreq;
    case MMC_CMD:
        return s->cmd | 0x40;
    case MMC_ARGH:
        return s->arg >> 16;
    case MMC_ARGL:
        return s->arg & 0xffff;
    case MMC_RES:
        if (s->resp_len < 9)
            return s->resp_fifo[s->resp_len ++];
        return 0;
    case MMC_RXFIFO:
        ret = 0;
        while (size-- && s->rx_len) {
            ret |= s->rx_fifo[s->rx_start++] << (size << 3);
            s->rx_start &= 0x1f;
            s->rx_len --;
        }
        s->intreq &= ~INT_RXFIFO_REQ;
        ls1gpa_mmci_fifo_update(s);
        return ret;
    case MMC_RDWAIT:
        return 0;
    case MMC_BLKS_REM:
        return s->numblk;
    default:
        hw_error("%s: Bad offset " REG_FMT "\n", __FUNCTION__, offset);
    }

    return 0;
}

static void ls1gpa_mmci_write(void *opaque,
                              hwaddr offset, uint64_t value, unsigned size)
{
    LS1GPA_MMCIState *s = (LS1GPA_MMCIState *) opaque;

    switch (offset) {
    case MMC_STRPCL:
        if (value & STRPCL_STRT_CLK) {
            s->status |= STAT_CLK_EN;
            s->intreq &= ~INT_CLK_OFF;

            if (s->cmdreq && !(s->cmdat & CMDAT_STOP_TRAN)) {
                s->status &= STAT_CLK_EN;
                ls1gpa_mmci_wakequeues(s);
            }
        }

        if (value & STRPCL_STOP_CLK) {
            s->status &= ~STAT_CLK_EN;
            s->intreq |= INT_CLK_OFF;
            s->active = 0;
        }

        ls1gpa_mmci_int_update(s);
        break;

    case MMC_CLKRT:
        s->clkrt = value & 7;
        break;

    case MMC_SPI:
        s->spi = value & 0xf;
        if (value & SPI_SPI_MODE)
            printf("%s: attempted to use card in SPI mode\n", __FUNCTION__);
        break;

    case MMC_CMDAT:
        s->cmdat = value & 0x3dff;
        s->active = 0;
        s->cmdreq = 1;
        if (!(value & CMDAT_STOP_TRAN)) {
            s->status &= STAT_CLK_EN;

            if (s->status & STAT_CLK_EN)
                ls1gpa_mmci_wakequeues(s);
        }

        ls1gpa_mmci_int_update(s);
        break;

    case MMC_RESTO:
        s->resp_tout = value & 0x7f;
        break;

    case MMC_RDTO:
        s->read_tout = value & 0xffff;
        break;

    case MMC_BLKLEN:
        s->blklen = value & 0xfff;
        break;

    case MMC_NUMBLK:
        s->numblk = value & 0xffff;
        break;

    case MMC_PRTBUF:
        if (value & PRTBUF_PRT_BUF) {
            s->tx_start ^= 32;
            s->tx_len = 0;
        }
        ls1gpa_mmci_fifo_update(s);
        break;

    case MMC_I_MASK:
        s->intmask = value & 0x1fff;
        ls1gpa_mmci_int_update(s);
        break;

    case MMC_CMD:
        s->cmd = value & 0x3f;
        break;

    case MMC_ARGH:
        s->arg &= 0x0000ffff;
        s->arg |= value << 16;
        break;

    case MMC_ARGL:
        s->arg &= 0xffff0000;
        s->arg |= value & 0x0000ffff;
        break;

    case MMC_TXFIFO:
        while (size-- && s->tx_len < 0x20)
            s->tx_fifo[(s->tx_start + (s->tx_len ++)) & 0x1f] =
                    (value >> (size << 3)) & 0xff;
        s->intreq &= ~INT_TXFIFO_REQ;
        ls1gpa_mmci_fifo_update(s);
        break;

    case MMC_RDWAIT:
    case MMC_BLKS_REM:
        break;

    default:
        hw_error("%s: Bad offset " REG_FMT "\n", __FUNCTION__, offset);
    }
}

static const MemoryRegionOps ls1gpa_mmci_ops = {
    .read = ls1gpa_mmci_read,
    .write = ls1gpa_mmci_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

LS1GPA_MMCIState *ls1gpa_mmci_init(MemoryRegion *sysmem,
                hwaddr base,
                BlockBackend *blk, qemu_irq irq,
                qemu_irq rx_dma, qemu_irq tx_dma)
{
    DeviceState *dev, *carddev;
    SysBusDevice *sbd;
    LS1GPA_MMCIState *s;
    Error *err = NULL;

    dev = qdev_create(NULL, TYPE_LS1GPA_MMCI);
    s = LS1GPA_MMCI(dev);
    sbd = SYS_BUS_DEVICE(dev);
    sysbus_mmio_map(sbd, 0, base);
    sysbus_connect_irq(sbd, 0, irq);
    qdev_connect_gpio_out_named(dev, "rx-dma", 0, rx_dma);
    qdev_connect_gpio_out_named(dev, "tx-dma", 0, tx_dma);

    /* Create and plug in the sd card */
    carddev = qdev_create(qdev_get_child_bus(dev, "sd-bus"), TYPE_SD_CARD);
    qdev_prop_set_drive(carddev, "drive", blk, &err);
    if (err) {
        error_report("failed to init SD card: %s", error_get_pretty(err));
        return NULL;
    }
    object_property_set_bool(OBJECT(carddev), true, "realized", &err);
    if (err) {
        error_report("failed to init SD card: %s", error_get_pretty(err));
        return NULL;
    }

    return s;
}

static void ls1gpa_mmci_set_inserted(DeviceState *dev, bool inserted)
{
    LS1GPA_MMCIState *s = LS1GPA_MMCI(dev);

    qemu_set_irq(s->inserted, inserted);
}

static void ls1gpa_mmci_set_readonly(DeviceState *dev, bool readonly)
{
    LS1GPA_MMCIState *s = LS1GPA_MMCI(dev);

    qemu_set_irq(s->readonly, readonly);
}

void ls1gpa_mmci_handlers(LS1GPA_MMCIState *s, qemu_irq readonly,
                          qemu_irq coverswitch)
{
    DeviceState *dev = DEVICE(s);

    s->readonly = readonly;
    s->inserted = coverswitch;

    ls1gpa_mmci_set_inserted(dev, sdbus_get_inserted(&s->sdbus));
    ls1gpa_mmci_set_readonly(dev, sdbus_get_readonly(&s->sdbus));
}

static void ls1gpa_mmci_reset(DeviceState *d)
{
    LS1GPA_MMCIState *s = LS1GPA_MMCI(d);

    s->status = 0;
    s->clkrt = 0;
    s->spi = 0;
    s->cmdat = 0;
    s->resp_tout = 0;
    s->read_tout = 0;
    s->blklen = 0;
    s->numblk = 0;
    s->intmask = 0;
    s->intreq = 0;
    s->cmd = 0;
    s->arg = 0;
    s->active = 0;
    s->bytesleft = 0;
    s->tx_start = 0;
    s->tx_len = 0;
    s->rx_start = 0;
    s->rx_len = 0;
    s->resp_len = 0;
    s->cmdreq = 0;
    memset(s->tx_fifo, 0, sizeof(s->tx_fifo));
    memset(s->rx_fifo, 0, sizeof(s->rx_fifo));
    memset(s->resp_fifo, 0, sizeof(s->resp_fifo));
}

static void ls1gpa_mmci_instance_init(Object *obj)
{
    LS1GPA_MMCIState *s = LS1GPA_MMCI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &ls1gpa_mmci_ops, s,
                          "ls1gpa-mmci", 0x00100000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_out_named(dev, &s->rx_dma, "rx-dma", 1);
    qdev_init_gpio_out_named(dev, &s->tx_dma, "tx-dma", 1);

    qbus_create_inplace(&s->sdbus, sizeof(s->sdbus),
                        TYPE_LS1GPA_MMCI_BUS, DEVICE(obj), "sd-bus");
}

static void ls1gpa_mmci_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_ls1gpa_mmci;
    dc->reset = ls1gpa_mmci_reset;
}

static void ls1gpa_mmci_bus_class_init(ObjectClass *klass, void *data)
{
    SDBusClass *sbc = SD_BUS_CLASS(klass);

    sbc->set_inserted = ls1gpa_mmci_set_inserted;
    sbc->set_readonly = ls1gpa_mmci_set_readonly;
}

static const TypeInfo ls1gpa_mmci_info = {
    .name = TYPE_LS1GPA_MMCI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LS1GPA_MMCIState),
    .instance_init = ls1gpa_mmci_instance_init,
    .class_init = ls1gpa_mmci_class_init,
};

static const TypeInfo ls1gpa_mmci_bus_info = {
    .name = TYPE_LS1GPA_MMCI_BUS,
    .parent = TYPE_SD_BUS,
    .instance_size = sizeof(SDBus),
    .class_init = ls1gpa_mmci_bus_class_init,
};

static void ls1gpa_mmci_register_types(void)
{
    type_register_static(&ls1gpa_mmci_info);
    type_register_static(&ls1gpa_mmci_bus_info);
}

type_init(ls1gpa_mmci_register_types)
