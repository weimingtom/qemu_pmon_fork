/*
 * ls1a RTC
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

#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "sysemu/sysemu.h"

enum {
    RTC_ID        = 0,
    RTC_LATCH     = 1,
    RTC_DATA_LOW  = 2,
    RTC_DATA_HIGH = 3
};

/*
 *
 * toy keep day moth hour sec etc.
 *
 * rtc reg decrease timercount and generate irq.
 *
 *
 */

enum {
TOYTRIM = 0x0,
TOYWRITE0 = 0x4,
TOYWRITE1 = 0x8,
TOYREAD0 = 0xc,
TOYREAD1 = 0x10,
TOYMATCH0 = 0x14,
TOYMATCH1 = 0x18,
TOYMATCH2 = 0x1c,
CNTRCTL = 0x20,
RTCTRIM = 0x40,
RTCWRITE0 = 0x44,
RTCREAD0 = 0x48,
RTCMATCH0 = 0x4c,
RTCMATCH1 = 0x50,
RTCMATCH2 = 0x54,
};

enum {
TOYEN = 1<<11,
RTCEN = 1<<13,
};


typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;
    int64_t offset;
    int64_t data;
    int tidx;
    uint32_t toymatch[3];
    uint32_t toytrim;
    uint32_t cntrctl;
    uint32_t rtctrim;
    uint32_t rtccount;
    uint32_t rtcmatch[3];
    qemu_irq irq[3];
    QEMUTimer *toy_timer;
} ls1a_rtc_state;

static void update_toy_timer(ls1a_rtc_state *s)
{
	int i;
	int imin,imax,inext;
	struct tm tm;
	uint64_t val, now, tmin, tmax, tnext;
	imin = imax = inext = 0;
	qemu_get_timedate(&tm, s->offset);
	tnext = tmin = tmax = now = \
	(tm.tm_sec << 0) | \
	(tm.tm_min << 6) | \
	(tm.tm_hour << 12) | \
	((tm.tm_mday-1) << 17) | \
	(tm.tm_mon << 22) |
	((tm.tm_year+1900) << 26);

	for(i=0;i<3;i++)
	{
	val = s->toymatch[i];
	if(tmin > val) {tmin = val; imin = i;}
	if(tmax < val) {tmax = val; imax = i;}
	if(val > now  && ((tnext > val) | (tnext == now))) { tnext = val;  inext = i; }
	}

	if(tmax <= now)
	{
		s->tidx = imin;
		timer_mod(s->toy_timer, (tmin-now)*1000);
	}
	else
	{
		s->tidx = inext;
		timer_mod(s->toy_timer, (tnext-now)*1000);
	}

}

static uint64_t ls1a_rtc_read(void *opaque, hwaddr offset, unsigned size)
{
    ls1a_rtc_state *s = (ls1a_rtc_state *)opaque;
    uint32_t val;
    struct tm tm;
    val = 0;
    switch (offset) {
    case TOYTRIM:
        val = s->toytrim;
	break;
    case TOYREAD0:
	qemu_get_timedate(&tm, s->offset);
	val = ((0)%10)  | \
	((tm.tm_sec) << 4) | \
	(((tm.tm_min)%60) << 10) | \
	(((tm.tm_hour)%24) << 16) | \
	(((tm.tm_mday-1) & 0x1f) << 21) | \
	((tm.tm_mon & 0x3f) << 26);
	break;
    case TOYREAD1:
	qemu_get_timedate(&tm, s->offset);
	val = tm.tm_year+1900;
	break;
    case TOYMATCH0:
	val = s->toymatch[0];
	break;
    case TOYMATCH1:
	val = s->toymatch[1];
	break;
    case TOYMATCH2:
	val = s->toymatch[2];
	break;
    case CNTRCTL:
	val = s->cntrctl;
	break;
    case RTCREAD0:
	val = s->rtccount;
	break;
    case RTCMATCH0:
	val = s->rtcmatch[0];
	break;
    case RTCMATCH1:
	val = s->rtcmatch[1];
	break;
    case RTCMATCH2:
	val = s->rtcmatch[2];
	break;
    default:
	val = 0;
	break;
    }
    return val;
}


static void ls1a_rtc_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    ls1a_rtc_state *s = (ls1a_rtc_state *)opaque;
    struct tm tm;

    switch (offset) {
    case TOYWRITE0:
	qemu_get_timedate(&tm, s->offset);

	tm.tm_sec = (val>>4)&0x3f;
	tm.tm_min = (val>>10)&0x3f;
	tm.tm_hour = (val >> 16)&0x1f;
	tm.tm_mday = ((val>>21) & 0x1f)+1;
	tm.tm_mon  = (val>>26) & 0x3f;

        s->offset = qemu_timedate_diff(&tm);
        break;
    case TOYWRITE1:
	qemu_get_timedate(&tm, s->offset);
	tm.tm_year = val - 1900;
        s->offset = qemu_timedate_diff(&tm);
        break;
    case TOYMATCH0:
	s->toymatch[0] = val;
	update_toy_timer(s);
	break;
    case TOYMATCH1:
	s->toymatch[1] = val;
	update_toy_timer(s);
	break;
    case TOYMATCH2:
	s->toymatch[2] = val;
	update_toy_timer(s);
	break;
    case CNTRCTL:
	s->cntrctl = val;
	break;
    case RTCWRITE0:
	s->rtccount = val;
	break;
    case RTCMATCH0:
	s->rtcmatch[0] = val;
	break;
    case RTCMATCH1:
	val = s->rtcmatch[1];
	break;
    case RTCMATCH2:
	val = s->rtcmatch[2];
	break;
    default:
        break;
    }
}

static void toy_timer(void *opaque)
{
	ls1a_rtc_state *s = opaque;
	uint64_t data;
	int year, month, day;
	struct tm tm;
	data = s->toymatch[s->tidx];
	year = (data >> 26) & 0x3f;
	month = (data >> 22) & 0x1f;
	day = (data >> 17) & 0x1f;

	qemu_get_timedate(&tm, s->offset);

	if(tm.tm_mday-1 == day && tm.tm_mon == month && tm.tm_year+1900 == year && (s->cntrctl & TOYEN))
	 qemu_irq_raise(s->irq[s->tidx]);
	update_toy_timer(s);
}

static const MemoryRegionOps ls1a_rtc_ops = {
    .read = ls1a_rtc_read,
    .write = ls1a_rtc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

#define TYPE_SYS_BUS_LS1ARTC "ls1a_rtc"

#define SYS_BUS_LS1ARTC(obj) \
    OBJECT_CHECK(ls1a_rtc_state, (obj), TYPE_SYS_BUS_LS1ARTC)

static int ls1a_rtc_init(SysBusDevice *dev)
{
    ls1a_rtc_state *d = SYS_BUS_LS1ARTC(dev);
    int i;

    memory_region_init_io(&d->iomem, NULL, &ls1a_rtc_ops, (void *)d, "ls1artc", 0x1000);

    for(i=0;i<3;i++)
    sysbus_init_irq(dev, &d->irq[3]);
    sysbus_init_mmio(dev, &d->iomem);

    d->toy_timer = timer_new_ms(rtc_clock, toy_timer, d);
    d->offset = 0;

    return 0;
}


static void ls1a_rtc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ls1a_rtc_init;
    dc->desc = "ls1a rtc";
}

static const TypeInfo ls1a_rtc_info = {
    .name          = "ls1a_rtc",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ls1a_rtc_state),
    .class_init    = ls1a_rtc_class_init,
};


static void ls1a_rtc_register_types(void)
{
    type_register_static(&ls1a_rtc_info);
}

type_init(ls1a_rtc_register_types)

