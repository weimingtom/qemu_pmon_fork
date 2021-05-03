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
#include "hw/loader.h"
#include "hw/sysbus.h"             /* SysBusDevice */
#include "hw/pci/pci.h"
#include "sysemu/dma.h"
#include "framebuffer.h"
#ifdef DEBUGPC
#include "exec/address-spaces.h"
#include "cpu.h"
#include "hw/mips/mips.h"
#include "hw/mips/cpudevs.h"
extern unsigned long long mypc;
#endif

struct ls1a_fb_states;
typedef struct ls1a_fb_state {
        struct ls1a_fb_states *fb;
        QemuConsole *con;
        MemoryRegionSection fbsection;
        ram_addr_t vram_offset;
        int head;
        int width;
        int height;
        int invalidate;
        int depth;
        int bypp;
        int enable;
        int config;
        int need_update;
        int index;
        int syncing;
        void *type;
} ls1a_fb_state ;

typedef struct ls1a_fb_states {
        SysBusDevice busdev;
        MemoryRegion iomem;

        int vram_size;
        struct ls1a_fb_state fb_con[2];

        union{
                MemoryRegion *root;
                void *root_ptr;
        };

        int reg[(0x1594-0x1240)/4];
} ls1a_fb_states;


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


static inline void ls2h_fb_size(ls1a_fb_state *s)
{
    		s->invalidate = 1;
		qemu_console_resize(s->con, s->width, s->height);
}

static void ls2h_fb_invalidate_screen(void *opaque)
{
	ls1a_fb_state *s = opaque;
	s->invalidate = 1;
	qemu_console_resize(s->con, s->width, s->height);
}

static void ls2h_fb_update_screen(void *opaque)
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
                                          s->fb->root,
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

static void ls2h_fb_reset(ls1a_fb_state *s)
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
	ls2h_fb_size(s);
}


#define LS2H_DC_REG_BASE				0

#define LS2H_FB_CFG_DVO_REG				(LS2H_DC_REG_BASE + 0x1240)
#define LS2H_FB_CFG_VGA_REG				(LS2H_DC_REG_BASE + 0x1250)
#define LS2H_FB_ADDR0_DVO_REG				(LS2H_DC_REG_BASE + 0x1260)
#define LS2H_FB_ADDR0_VGA_REG				(LS2H_DC_REG_BASE + 0x1270)
#define LS2H_FB_STRI_DVO_REG				(LS2H_DC_REG_BASE + 0x1280)
#define LS2H_FB_STRI_VGA_REG				(LS2H_DC_REG_BASE + 0x1290)
#define LS2H_FB_HDISPLAY_DVO_REG			(LS2H_DC_REG_BASE + 0x1400)
#define LS2H_FB_HDISPLAY_VGA_REG			(LS2H_DC_REG_BASE + 0x1410)
#define LS2H_FB_VDISPLAY_DVO_REG			(LS2H_DC_REG_BASE + 0x1480)
#define LS2H_FB_VDISPLAY_VGA_REG			(LS2H_DC_REG_BASE + 0x1490)

static void ls2h_fb_writel (void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
 ls1a_fb_states *d = opaque;
 ls1a_fb_state *s;
 if (addr >= 0x1240 && (addr - 0x1240) < sizeof(d->reg))
	d->reg[(addr - 0x1240)/4] = val;

 s = (addr &0x10)?&d->fb_con[1]:&d->fb_con[0];
 switch(addr)
 {
 case LS2H_FB_CFG_DVO_REG: 
 case LS2H_FB_CFG_VGA_REG:
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
                 ls2h_fb_size(s);
         }
         break;
 case LS2H_FB_HDISPLAY_DVO_REG:
 case LS2H_FB_HDISPLAY_VGA_REG:
         s->width = val&0xffff;
         s->need_update = 1;
         ls2h_fb_size(s);
         break;
 case LS2H_FB_VDISPLAY_DVO_REG:
 case LS2H_FB_VDISPLAY_VGA_REG:
         s->height = val&0xffff;
         s->need_update = 1;
         ls2h_fb_size(s);
         break;

 case LS2H_FB_ADDR0_DVO_REG:
 case LS2H_FB_ADDR0_VGA_REG:
         s->vram_offset = val;
         s->need_update = 1;
         ls2h_fb_size(s);
         break;
 }

#ifdef DEBUGPC
 printf("%s 0x%llx 0x%llx pc 0x%llx\n", __FUNCTION__, (long long)addr, (long long) val, (long long)mypc);
 MIPSCPU *cpu = MIPS_CPU(current_cpu);
 CPUMIPSState *env = &cpu->env;
 env->active_tc.PC = mypc;
 do_raise_exception_err(env, EXCP_DEBUG, 0, mypc);
#endif
}

static uint64_t ls2h_fb_readl (void *opaque, hwaddr addr, unsigned size)
{
 ls1a_fb_states *s = opaque;
 if (addr >= 0x1240 && (addr - 0x1240) < sizeof(s->reg))
	 return	s->reg[(addr - 0x1240)/4];
    return 0;
}

static const MemoryRegionOps ls2h_fb_ops = {
    .read = ls2h_fb_readl,
    .write = ls2h_fb_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

#define TYPE_SYS_BUS_LS2HFB "ls2h_fb"

#define SYS_BUS_LS2HFB(obj) \
    OBJECT_CHECK(ls1a_fb_states, (obj), TYPE_SYS_BUS_LS2HFB)

static const GraphicHwOps ls2hfb_fb_ops = {
    .invalidate  = ls2h_fb_invalidate_screen,
    .gfx_update  = ls2h_fb_update_screen,
};

static int ls2h_fb_sysbus_init(SysBusDevice *dev)
{

    ls1a_fb_states *d = SYS_BUS_LS2HFB(dev);

    memory_region_init_io(&d->iomem, NULL, &ls2h_fb_ops, (void *)d, "ls2h fb", 0x10000);

    sysbus_init_mmio(dev, &d->iomem);
    d->vram_size = 0;
    d->root = sysbus_address_space(dev);
    
    d->fb_con[0].con = graphic_console_init(DEVICE(dev), 0, &ls2hfb_fb_ops, &d->fb_con[0]);
    d->fb_con[1].con = graphic_console_init(DEVICE(dev), 1, &ls2hfb_fb_ops, &d->fb_con[1]);
    d->fb_con[0].fb = d;
    d->fb_con[1].fb = d;
    ls2h_fb_reset(&d->fb_con[0]);
    ls2h_fb_reset(&d->fb_con[1]);

    return 0;
}

static Property ls2h_fb_properties[] = {
    DEFINE_PROP_PTR("root", ls1a_fb_states, root_ptr),
    DEFINE_PROP_END_OF_LIST(),
};

static void ls2h_fb_sysbus_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ls2h_fb_sysbus_init;
    dc->desc = "ls2h fb";
    dc->props = ls2h_fb_properties;
}

static const TypeInfo ls2h_fb_sysbus_info = {
    .name          = "ls2h_fb",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ls1a_fb_states),
    .class_init    = ls2h_fb_sysbus_class_init,
};


static void ls2h_fb_sysbus_register_types(void)
{
    type_register_static(&ls2h_fb_sysbus_info);
}

type_init(ls2h_fb_sysbus_register_types)

//-------------
typedef struct ls1a_fb_pci_state {
    PCIDevice dev;
    ls1a_fb_states dc;
} ls1a_fb_pci_state;

/* PCI interface */
#define LS2KDC_VENDOR_ID  0x0014
#define LS2KDC_DEVICE_ID  0x7a06

static void ls1a_fb_pci_init(PCIDevice *pci_dev, Error **errp)
{
    ls1a_fb_pci_state *d = DO_UPCAST(ls1a_fb_pci_state, dev, pci_dev);
    uint8_t *pci_conf;

    pci_conf = d->dev.config;

    pci_config_set_vendor_id(pci_conf, LS2KDC_VENDOR_ID);
    pci_config_set_device_id(pci_conf, LS2KDC_DEVICE_ID);
    pci_conf[0x04] = 0x07; /* command = I/O space, Bus Master */
    pci_config_set_class(pci_conf, PCI_CLASS_DISPLAY_VGA);
    pci_conf[PCI_HEADER_TYPE] = PCI_HEADER_TYPE_NORMAL; /* header_type */
    pci_conf[PCI_INTERRUPT_PIN] = 1; /* interrupt pin A */
    pci_conf[0x34] = 0xdc;


    //d->dc.root = get_system_memory();
    d->dc.root = pci_address_space(pci_dev);

    memory_region_init_io(&d->dc.iomem, NULL, &ls2h_fb_ops, (void *)&d->dc, "ls2h fb", 0x10000);
    pci_register_bar(&d->dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->dc.iomem);

    d->dc.vram_size = 0;
    
    d->dc.fb_con[0].con = graphic_console_init(DEVICE(pci_dev), 0, &ls2hfb_fb_ops, &d->dc.fb_con[0]);
    d->dc.fb_con[1].con = graphic_console_init(DEVICE(pci_dev), 1, &ls2hfb_fb_ops, &d->dc.fb_con[1]);
    d->dc.fb_con[0].fb = &d->dc;
    d->dc.fb_con[1].fb = &d->dc;
    ls2h_fb_reset(&d->dc.fb_con[0]);
    ls2h_fb_reset(&d->dc.fb_con[1]);

}


static Property ls1a_fb_pci_properties[] = {
    DEFINE_PROP_PTR("root", ls1a_fb_pci_state, dc.root_ptr),
    DEFINE_PROP_END_OF_LIST(),
};

static void ls1a_fb_pci_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = ls1a_fb_pci_init;
    k->vendor_id = LS2KDC_VENDOR_ID;
    k->device_id = LS2KDC_DEVICE_ID;
    k->revision = 0x03;
    k->class_id = PCI_CLASS_DISPLAY_VGA;
    dc->desc = "ls1a dc pci";
    dc->props = ls1a_fb_pci_properties;
}

static const TypeInfo ls1a_fb_pci_info = {
    .name          = "pci_ls2h_fb",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(ls1a_fb_pci_state),
    .class_init    = ls1a_fb_pci_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { }
    },
};

static void ls1a_fb_pci_register_types(void)
{
    type_register_static(&ls1a_fb_pci_info);
}

type_init(ls1a_fb_pci_register_types)
