/*
 * loongson 1a ACPI implementation
 * Author: Zeng Lu <zenglu@ict.ac.cn>
 *
 * Copyright (c) 2006 Fabrice Bellard
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/i386/pc.h"
#include "hw/pci/pci.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/smbus.h"
#include <sysemu/sysemu.h>

//#define DEBUG

/* i82731AB (PIIX4) compatible power management function */
#define PM_FREQ 3579545

#define ACPI_DBG_IO_ADDR  0xb044

typedef struct PIIX4PMState {
    SysBusDevice busdev;
    PCIDevice dev;
    MemoryRegion iomem;
    uint16_t pmsts;
    uint16_t pmen;
    uint16_t pmcntrl;
    uint8_t apmc;
    uint8_t apms;
    QEMUTimer *tmr_timer;
    int64_t tmr_overflow_time;
    I2CBus *smbus;
    uint8_t smb_stat;
    uint8_t smb_ctl;
    uint8_t smb_cmd;
    uint8_t smb_addr;
    uint8_t smb_data0;
    uint8_t smb_data1;
    uint8_t smb_data[32];
    uint8_t smb_index;
    qemu_irq irq;
} PIIX4PMState;

static const VMStateDescription vmstate_ls1a_acpi = {
    .name = "ls1a_acpi",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT16(pmsts, PIIX4PMState),
        VMSTATE_END_OF_LIST()
    }
};

#define RSM_STS (1 << 15)
#define PWRBTN_STS (1 << 8)
#define RTC_EN (1 << 10)
#define PWRBTN_EN (1 << 8)
#define GBL_EN (1 << 5)
#define TMROF_EN (1 << 0)

#define SCI_EN (1 << 0)

#define SUS_EN (1 << 13)

#define ACPI_ENABLE 0xf1
#define ACPI_DISABLE 0xf0

#define SMBHSTSTS 0x00
#define SMBHSTCNT 0x02
#define SMBHSTCMD 0x03
#define SMBHSTADD 0x04
#define SMBHSTDAT0 0x05
#define SMBHSTDAT1 0x06
#define SMBBLKDAT 0x07


static uint32_t get_pmtmr(PIIX4PMState *s)
{
    uint32_t d;
    d = muldiv64(qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), PM_FREQ, NANOSECONDS_PER_SECOND);
    return d & 0xffffff;
}

static int get_pmsts(PIIX4PMState *s)
{
    int64_t d;
    d = muldiv64(qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), PM_FREQ, NANOSECONDS_PER_SECOND);
    if (d >= s->tmr_overflow_time)
        s->pmsts |= TMROF_EN;
    return s->pmsts;
}

static void pm_update_sci(PIIX4PMState *s)
{
    int sci_level, pmsts;
    int64_t expire_time;

    pmsts = get_pmsts(s);
    sci_level = (((pmsts & s->pmen) &
                  (RTC_EN | PWRBTN_EN | GBL_EN | TMROF_EN)) != 0);
    qemu_set_irq(s->irq, sci_level);
    /* schedule a timer interruption if needed */
    if ((s->pmen & TMROF_EN) && !(pmsts & TMROF_EN)) {
        expire_time = muldiv64(s->tmr_overflow_time, NANOSECONDS_PER_SECOND, PM_FREQ);
        timer_mod(s->tmr_timer, expire_time);
    } else {
        timer_del(s->tmr_timer);
    }
}

static void pm_tmr_timer(void *opaque)
{
    PIIX4PMState *s = opaque;
    pm_update_sci(s);
}

static void pm_mem_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    PIIX4PMState *s = opaque;
    addr &= 0x3f;
    switch(addr) {
    case 0x00:
        {
            int64_t d;
            int pmsts;
            pmsts = get_pmsts(s);
            if (pmsts & val & TMROF_EN) {
                /* if TMRSTS is reset, then compute the new overflow time */
                d = muldiv64(qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), PM_FREQ,
                             NANOSECONDS_PER_SECOND);
                s->tmr_overflow_time = (d + 0x800000LL) & ~0x7fffffLL;
            }
            s->pmsts &= ~val;
            pm_update_sci(s);
        }
        break;
    case 0x04:
        s->pmen = val;
        pm_update_sci(s);
        break;
    case 0x08:
        {
            int sus_typ;
            s->pmcntrl = val & ~(SUS_EN);
            if (val & SUS_EN) {
                /* change suspend type */
                sus_typ = (val >> 10) & 7;
                switch(sus_typ) {
                case 0: /* soft power off */
                    qemu_system_shutdown_request();
                    break;
                case 1:
                    /* RSM_STS should be set on resume. Pretend that resume
                       was caused by power button */
                    s->pmsts |= (RSM_STS | PWRBTN_STS);
                    qemu_system_reset_request();
                default:
                    break;
                }
            }
        }
        break;
    default:
        break;
    }
#ifdef DEBUG
    printf("PM writew port=0x%04x val=0x%04x\n", (unsigned)addr, (unsigned)val);
#endif
}


static uint64_t pm_mem_read(void *opaque, hwaddr addr, unsigned size)
{
    PIIX4PMState *s = opaque;
    uint32_t val;

    addr &= 0x3f;
    switch(addr) {
    case 0x00:
        val = get_pmsts(s);
        break;
    case 0x02:
        val = s->pmen;
        break;
    case 0x04:
        val = s->pmcntrl;
        break;
    case 0x08:
        val = get_pmtmr(s);
        break;
    default:
        val = 0;
        break;
    }
#ifdef DEBUG
    printf("PM read port=0x%04x val=0x%04x\n", (unsigned)addr, val);
#endif
    return val;
}

static const MemoryRegionOps pm_ops = {
        .read = pm_mem_read,
        .write = pm_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static void piix4_powerdown(void *opaque, int irq, int power_failing)
{
    PIIX4PMState *s = opaque;

    if (!s) {
        qemu_system_shutdown_request();
    } else if (s->pmen & PWRBTN_EN) {
        s->pmsts |= PWRBTN_EN;
        pm_update_sci(s);
    }
}

static qemu_irq ls1a_acpi_system_powerdown;

static void ls1a_acpi_powerdown_req(Notifier *n, void *opaque)
{
    qemu_irq_raise(ls1a_acpi_system_powerdown);
}

static Notifier ls1a_acpi_system_powerdown_notifier = {
    .notify = ls1a_acpi_powerdown_req
};


//---------------
#define TYPE_SYS_BUS_LS1AACPI "ls1a_acpi"

#define SYS_BUS_LS1AACPI(obj) \
    OBJECT_CHECK(PIIX4PMState, (obj), TYPE_SYS_BUS_LS1AACPI)

static int ls1a_acpi_sysbus_init(SysBusDevice *dev)
{
    PIIX4PMState *d = SYS_BUS_LS1AACPI(dev);

    memory_region_init_io(&d->iomem, NULL, &pm_ops, (void *)d, "ls1a_acpi", 0x1000);

    sysbus_init_mmio(dev, &d->iomem);
    sysbus_init_irq(dev, &d->irq);
    d->tmr_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, pm_tmr_timer, d);

    ls1a_acpi_system_powerdown = *qemu_allocate_irqs(piix4_powerdown, d, 1);
    qemu_register_powerdown_notifier(&ls1a_acpi_system_powerdown_notifier);

    return 0;
}

static void ls1a_acpi_sysbus_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ls1a_acpi_sysbus_init;
    dc->desc = "ls1a ls1a_acpi";
}

static const TypeInfo ls1a_acpi_sysbus_info = {
    .name          = "ls1a_acpi",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PIIX4PMState),
    .class_init    = ls1a_acpi_sysbus_class_init,
};


static void ls1a_acpi_sysbus_register_types(void)
{
    type_register_static(&ls1a_acpi_sysbus_info);
}

type_init(ls1a_acpi_sysbus_register_types)
