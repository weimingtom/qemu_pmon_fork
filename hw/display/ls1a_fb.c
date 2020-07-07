/*
 * QEMU loongson 1a framebuffer emulation
 *
 * Copyright (c) 2013 qiaochong@loongson.cn
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
#include "ui/console.h"
#include "ui/pixel_ops.h"
#include "hw/sysbus.h"             /* SysBusDevice */
#include "sysemu/dma.h"
#include "framebuffer.h"

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;

    int width;
    int height;
    int invalidate;
    int depth;
    int bypp;
    int enable;
    int config;
    int need_update;
    void *type;

    QemuConsole *con;
    int vram_size;
    ram_addr_t vram_offset;
    MemoryRegionSection fbsection;

    union{
	    MemoryRegion *root;
	    void *root_ptr;
    };

    int index;
    int syncing;
    int reg[0x354/4];
} ls1a_fb_state;


static const VMStateDescription vmstate_ls1a_fb = {
    .name = "ls1a_fb",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_INT32(width, ls1a_fb_state),
        VMSTATE_END_OF_LIST()
    }
};


#define BITS 8
#include "ls1a_fb_template.h"
#define BITS 15
#include "ls1a_fb_template.h"
#define BITS 16
#include "ls1a_fb_template.h"
#define BITS 24
#include "ls1a_fb_template.h"
#define BITS 32
#include "ls1a_fb_template.h"


static inline void ls1a_fb_size(ls1a_fb_state *s)
{
    		s->invalidate = 1;
		qemu_console_resize(s->con, s->width, s->height);
}

static void ls1a_fb_invalidate_screen(void *opaque)
{
	ls1a_fb_state *s = opaque;
	s->invalidate = 1;
	qemu_console_resize(s->con, s->width, s->height);
}

static void ls1a_fb_update_screen(void *opaque)
{
    ls1a_fb_state *s = opaque;
    DisplaySurface *surface = qemu_console_surface(s->con);

    int first = 0;
    int last = 0;
    drawfn fn;


    int dest_width = s->width;

    switch (surface_bits_per_pixel(surface)) {
    case 0:
        return;
    case 8:
        fn = draw_line_8;
        break;
    case 15:
        fn = draw_line_15;
        dest_width *= 2;
        break;
    case 16:
        fn = draw_line_16;
        dest_width *= 2;
        break;
    case 24:
        fn = draw_line_24;
        dest_width *= 3;
        break;
    case 32:
        fn = draw_line_32;
        dest_width *= 4;
        break;
    default:
        hw_error("milkymist_vgafb: bad color depth\n");
        break;
    }

    if (s->invalidate) {
        framebuffer_update_memory_section(&s->fbsection,
                                          s->root,
                                          s->vram_offset,
                                          s->width, s->width * s->bypp);
    }

    framebuffer_update_display(surface, &s->fbsection,
                               s->width,
                               s->height,
                               s->width * s->bypp,
                               dest_width,
                               0,
                               s->invalidate,
                               fn,
                               s,
                               &first, &last);

    if (first >= 0) {
        dpy_gfx_update(s->con, 0, first, s->width, last - first + 1);
    }
    s->invalidate = 0;
}

static void ls1a_fb_reset(ls1a_fb_state *s)
{
	int width=800,height=600,depth=16;
	unsigned int fb_offset=0;

	if(getenv("SIMPLEVGA"))
	{
		if(sscanf(getenv("SIMPLEVGA"),"%dx%d-%d:0x%x",&width,&height,&depth,&fb_offset)<3)
		{
			printf("usage: SIMPLEVGA=800x600-16:0xc3000\n");
			exit(1);
		}
	}
	s->index = 0;
	s->need_update = 1;
	s->enable = 0;
	s->config = 0;
	s->width = width;
	s->height = height;
	s->depth = depth;

	s->vram_offset = fb_offset;
	s->bypp = (s->depth + 7) >> 3;
	s->syncing = 0;
	ls1a_fb_size(s);
}


enum{
OF_BUF_CONFIG=0,
OF_BUF_ADDR=0x20,
OF_BUF_STRIDE=0x40,
OF_BUF_ORIG=0x60,
OF_DITHER_CONFIG=0x120,
OF_DITHER_TABLE_LOW=0x140,
OF_DITHER_TABLE_HIGH=0x160,
OF_PAN_CONFIG=0x180,
OF_PAN_TIMING=0x1a0,
OF_HDISPLAY=0x1c0,
OF_HSYNC=0x1e0,
OF_VDISPLAY=0x240,
OF_VSYNC=0x260,
OF_DBLBUF=0x340,
};


static void ls1a_fb_writel (void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
 ls1a_fb_state *s = opaque;
 if (addr < sizeof(s->reg))
	s->reg[addr/4] = val;
addr=0x1c301240 + addr;
 switch(addr)
 {
  case 0x1c301240+OF_BUF_CONFIG:
  case 0x1c301250+OF_BUF_CONFIG:
	if(val&0x100)
	{
	  switch(val&7)
	  {
		  case 4:
			  s->depth = 32;
			  break;
		  case 3:
			  s->depth = 16;
			  break;
		  case 2:
			  s->depth = 15;
			  break;
		  default:
			  s->depth = 16;
			  break;
	  }
	  s->bypp = (s->depth + 7) >> 3;
	  s->need_update = 1;
	ls1a_fb_size(s);
      }
  break;
  case 0x1c301240+OF_HDISPLAY:
  case 0x1c301250+OF_HDISPLAY:
	s->width = val&0xffff;
	s->need_update = 1;
	ls1a_fb_size(s);
	break;
  case 0x1c301240+OF_VDISPLAY:
  case 0x1c301250+OF_VDISPLAY:
	s->height = val&0xffff;
	s->need_update = 1;
	ls1a_fb_size(s);
	break;

  case 0x1c301240+OF_BUF_ADDR:
  case 0x1c301250+OF_BUF_ADDR:
  s->vram_offset = val;
  s->need_update = 1;
	ls1a_fb_size(s);
  break;
 }
}

static uint64_t ls1a_fb_readl (void *opaque, hwaddr addr, unsigned size)
{
ls1a_fb_state *s = opaque;
 if (addr < sizeof(s->reg))
	return s->reg[addr/4];
    return 0;
}

static const MemoryRegionOps ls1a_fb_ops = {
    .read = ls1a_fb_readl,
    .write = ls1a_fb_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

#define TYPE_SYS_BUS_LS1AFB "ls1a_fb"

#define SYS_BUS_LS1AFB(obj) \
    OBJECT_CHECK(ls1a_fb_state, (obj), TYPE_SYS_BUS_LS1AFB)

static const GraphicHwOps ls1afb_fb_ops = {
    .invalidate  = ls1a_fb_invalidate_screen,
    .gfx_update  = ls1a_fb_update_screen,
};

static int ls1a_fb_sysbus_init(SysBusDevice *dev)
{
    ls1a_fb_state *d = SYS_BUS_LS1AFB(dev);

    memory_region_init_io(&d->iomem, NULL, &ls1a_fb_ops, (void *)d, "ls1a fb", 0x400);

    sysbus_init_mmio(dev, &d->iomem);
    d->vram_size = 0;
    d->vram_offset = 0;
    d->root = sysbus_address_space(dev);
    
    d->con = graphic_console_init(DEVICE(dev), 0, &ls1afb_fb_ops, d);
    ls1a_fb_reset(d);

    return 0;
}

static Property ls1a_fb_properties[] = {
    DEFINE_PROP_PTR("root", ls1a_fb_state, root_ptr),
    DEFINE_PROP_END_OF_LIST(),
};

static void ls1a_fb_sysbus_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ls1a_fb_sysbus_init;
    dc->desc = "ls1a fb";
    dc->props = ls1a_fb_properties;
}

static const TypeInfo ls1a_fb_sysbus_info = {
    .name          = "ls1a_fb",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ls1a_fb_state),
    .class_init    = ls1a_fb_sysbus_class_init,
};


static void ls1a_fb_sysbus_register_types(void)
{
    type_register_static(&ls1a_fb_sysbus_info);
}

type_init(ls1a_fb_sysbus_register_types)
