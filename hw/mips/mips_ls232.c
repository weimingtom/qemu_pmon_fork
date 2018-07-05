/*
 * QEMU loongson 1b develop board emulation
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
#include "hw/mips/mips.h"
#include "hw/mips/cpudevs.h"
#include "hw/i386/pc.h"
#include "hw/char/serial.h"
#include "hw/isa/isa.h"
#include "net/net.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/block/flash.h"
#include "qemu/log.h"
#include "hw/mips/bios.h"
#include "hw/ide.h"
#include "hw/loader.h"
#include "elf.h"
#include "hw/sysbus.h"
#include "hw/net/synopGMAC.h"
#include "sysemu/blockdev.h"
#include "hw/ssi/ssi.h"
#include "hw/i2c/i2c.h"
#include "exec/address-spaces.h"

#define PHYS_TO_VIRT(x) ((x) | ~(target_ulong)0x7fffffff)

#define VIRT_TO_PHYS_ADDEND (-((int64_t)(int32_t)0x80000000))

#define MAX_IDE_BUS 2
#define TARGET_REALPAGE_MASK (TARGET_PAGE_MASK<<2)

static const int ide_iobase[2] = { 0x1f0, 0x170 };
static const int ide_iobase2[2] = { 0x3f6, 0x376 };
static const int ide_irq[2] = { 14, 15 };

/* i8254 PIT is attached to the IRQ0 at PIC i8259 */

static struct _loaderparams {
	int ram_size;
	const char *kernel_filename;
	const char *kernel_cmdline;
	const char *initrd_filename;
} loaderparams;

static int clkreg[2];
static MemoryRegion *ddrcfg_iomem;
static int reg0420[2]={0,0x100000};

static void mips_qemu_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case 0x1fe78030:
		case 0x1fe78034:
			clkreg[(addr - 0x1fe78030)/4] = val;
			break;

		case 0x1fd00420:
			reg0420[0] = val;
			break;
		case 0x1fd00424:

			if(val&0x100000 && !(reg0420[1]&0x100000))
			{
				memory_region_del_subregion(get_system_memory(), ddrcfg_iomem);
			}

			if(!(val&0x100000) && (reg0420[1]&0x100000))
			{
				memory_region_add_subregion_overlap(get_system_memory(), 0x0ffffe00&TARGET_PAGE_MASK, ddrcfg_iomem, 1);
			}

			reg0420[1] = val;

			break;
	}
}

static uint64_t mips_qemu_readl (void *opaque, hwaddr addr, unsigned size)
{
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case 0x1fe78030:
		case 0x1fe78034:
			return clkreg[(addr - 0x1fe78030)/4];
			break;
		case 0x0ffffe10:
			return 1;
			break;
		case 0x0ffffef0:
			return 0x100000;
			break;
		case 0x0ffffef2:
			return 0x10;
			break;
	}
	return 0;
}

static const MemoryRegionOps mips_qemu_ops = {
    .read = mips_qemu_readl,
    .write = mips_qemu_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


typedef struct ResetData {
	MIPSCPU *cpu;
	uint64_t vector;
} ResetData;

static int64_t load_kernel(void)
{
	int64_t entry, kernel_low, kernel_high;
	long kernel_size, initrd_size, params_size;
	char *params_buf;
	ram_addr_t initrd_offset;
	int ret;

	if(getenv("BOOTROM"))
	{
		initrd_size = load_image_targphys(loaderparams.kernel_filename,
				strtoul(getenv("BOOTROM"),0,0),ram_size); //qemu_get_ram_ptr
		return 0;
	}
	kernel_size = load_elf(loaderparams.kernel_filename, cpu_mips_kseg0_to_phys, NULL,
			(uint64_t *)&entry, (uint64_t *)&kernel_low,
			(uint64_t *)&kernel_high,0,EM_MIPS, 1, 0);
	if (kernel_size >= 0) {
		if ((entry & ~0x7fffffffULL) == 0x80000000)
			entry = (int32_t)entry;
	} else {
		fprintf(stderr, "qemu: could not load kernel '%s'\n",
				loaderparams.kernel_filename);
		exit(1);
	}

	/* load initrd */
	initrd_size = 0;
	initrd_offset = 0;
	if (loaderparams.initrd_filename) {
		initrd_size = get_image_size (loaderparams.initrd_filename);
		if (initrd_size > 0) {
			initrd_offset = (kernel_high + ~TARGET_REALPAGE_MASK) & TARGET_REALPAGE_MASK;
			if (initrd_offset + initrd_size > ram_size) {
				fprintf(stderr,
						"qemu: memory too small for initial ram disk '%s'\n",
						loaderparams.initrd_filename);
				exit(1);
			}
			initrd_size = load_image_targphys(loaderparams.initrd_filename,
					initrd_offset,ram_size-initrd_offset); //qemu_get_ram_ptr
		}
		if (initrd_size == (target_ulong) -1) {
			fprintf(stderr, "qemu: could not load initial ram disk '%s'\n",
					loaderparams.initrd_filename);
			exit(1);
		}
	}

	/* Store command line.  */
	params_size = 264;
	params_buf = g_malloc(params_size);



#define BOOTPARAM_PHYADDR ((64 << 20) - 264)
#define BOOTPARAM_ADDR (0x80000000+BOOTPARAM_PHYADDR)
	// should set argc,argv
	//env->gpr[REG][env->current_tc]
	{
		char memenv[32];
		int i;
		unsigned int *parg_env=(void *)params_buf;
		/*
		 * pram buf like this:
		 *argv[0] argv[1] 0 env[0] env[1] ...env[i] ,0, argv[0]'s data , argv[1]'s data ,env[0]'data,...,env[i]'s dat,0
		 */
		
		sprintf(memenv,"%d",loaderparams.ram_size>0x10000000?256:(loaderparams.ram_size>>20));
		setenv("ENV_memsize", memenv, 1);
		sprintf(memenv,"%d",loaderparams.ram_size>0x10000000?(loaderparams.ram_size>>20)-256:0);
		setenv("ENV_highmemsize", memenv, 1);
		setenv("ENV_cpuclock", "200000000", 0);
		setenv("ENV_busclock", "33333333", 0);

		//*count user special env
		for(ret=0,i=0;environ[i];i++)
			if(!strncmp(environ[i],"ENV_",4))ret+=4;

		//jump over argv and env area
		ret +=(3+1)*4;
		//argv0
		*parg_env++=BOOTPARAM_ADDR+ret;
		ret +=1+snprintf(params_buf+ret,256-ret,"g");
		//argv1
		*parg_env++=BOOTPARAM_ADDR+ret;
		if (initrd_size > 0) {
			ret +=1+snprintf(params_buf+ret,256-ret, "rd_start=0x" TARGET_FMT_lx " rd_size=%li %s",
					PHYS_TO_VIRT((uint32_t)initrd_offset),
					initrd_size, loaderparams.kernel_cmdline);
		} else {
			ret +=1+snprintf(params_buf+ret, 256-ret, "%s", loaderparams.kernel_cmdline);
		}
		//argv2
		*parg_env++=0;


		for(i=0;environ[i];i++)
		{
			if(!strncmp(environ[i],"ENV_",4)){
				*parg_env++=BOOTPARAM_ADDR+ret;
				ret +=1+snprintf(params_buf+ret,256-ret,"%s",&environ[i][4]);
			}
		}
		*parg_env++=0;
		rom_add_blob_fixed("params", params_buf, params_size,
				BOOTPARAM_PHYADDR);

	}
	return entry;
}
static void main_cpu_reset(void *opaque)
{
	ResetData *s = (ResetData *)opaque;
	CPUMIPSState *env = &s->cpu->env;

	cpu_reset(CPU(s->cpu));
	env->CP0_IntCtl = 0xfc000000;
	env->active_tc.PC = s->vector;
	env->active_tc.gpr[4]=2;
	env->active_tc.gpr[5]=0x80000000+BOOTPARAM_PHYADDR;
	env->active_tc.gpr[6]=0x80000000+BOOTPARAM_PHYADDR +12;
}



static const int sector_len = 32 * 1024;


static void mips_ls232_init (MachineState *machine)
{
	ram_addr_t ram_size = machine->ram_size;
	const char *kernel_filename = machine->kernel_filename;
	const char *kernel_cmdline = machine->kernel_cmdline;
	const char *initrd_filename = machine->initrd_filename;
	char *filename;
	MemoryRegion *address_space_mem = get_system_memory();
	MemoryRegion *ram = g_new(MemoryRegion, 1);
	MemoryRegion *bios;
	int bios_size;
	MIPSCPU *cpu;
	CPUMIPSState *env;
	ResetData *reset_info;
	DriveInfo *flash_dinfo=NULL;
	int ddr2config = 0;


	/* init CPUs */
	cpu = MIPS_CPU(cpu_create(machine->cpu_type));
		env = &cpu->env;
		reset_info = g_malloc0(sizeof(ResetData));
		reset_info->cpu = cpu;
		reset_info->vector = env->active_tc.PC;
		qemu_register_reset(main_cpu_reset, reset_info);

		/* allocate RAM */
	memory_region_init_ram(ram, NULL, "mips_r4k.ram", ram_size, &error_fatal);

	memory_region_add_subregion(address_space_mem, 0, ram);

	//memory_region_init_io(iomem, &mips_qemu_ops, NULL, "mips-qemu", 0x10000);
	//memory_region_add_subregion(address_space_mem, 0x1fbf0000, iomem);

    /* Try to load a BIOS image. If this fails, we continue regardless,
       but initialize the hardware ourselves. When a kernel gets
       preloaded we also initialize the hardware, since the BIOS wasn't
       run. */
    if (bios_name == NULL)
        bios_name = BIOS_FILENAME;
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    if (filename) {
        bios_size = get_image_size(filename);
    } else {
        bios_size = -1;
    }

    if ((bios_size > 0) && (bios_size <= BIOS_SIZE)) {
        bios = g_new(MemoryRegion, 1);
        memory_region_init_ram(bios, NULL, "mips_r4k.bios", BIOS_SIZE, &error_fatal);
        memory_region_set_readonly(bios, true);
        memory_region_add_subregion(get_system_memory(), 0x1fc00000, bios);

        load_image_targphys(filename, 0x1fc00000, BIOS_SIZE);
	ddr2config = 1;
    } else if ((flash_dinfo = drive_get_next(IF_PFLASH)))
	ddr2config = 1;
    else {
	/* not fatal */
        fprintf(stderr, "qemu: Warning, could not load MIPS bios '%s'\n",
		bios_name);
    }
    if (filename) {
        g_free(filename);
    }

    if (kernel_filename) {
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        reset_info->vector = load_kernel();
    }


	/* Init CPU internal devices */
	cpu_mips_irq_init_cpu(cpu);
	cpu_mips_clock_init(cpu);




	if (serial_hds[0])
		serial_mm_init(address_space_mem, 0x1fe40000, 0, env->irq[3],115200,serial_hds[0], DEVICE_NATIVE_ENDIAN);


	//sysbus_create_simple("ls1a_fb", 0x1c301240, NULL);



	{
		DeviceState *dev,*dev1;
		void *bus;
		qemu_irq cs_line;
		dev=sysbus_create_simple("ls1a_spi",0x1fe80000, env->irq[4]);
		bus = qdev_get_child_bus(dev, "ssi");
		if(flash_dinfo)
		{
			dev1 = ssi_create_slave_no_init(bus, "spi-flash");
			qdev_prop_set_drive(dev1, "drive", blk_by_legacy_dinfo(flash_dinfo), &error_fatal);
			qdev_prop_set_uint32(dev1, "size", 0x100000);
			qdev_prop_set_uint64(dev1, "addr", 0x1fc00000);
			qdev_init_nofail(dev1);
		}
		else dev1 = ssi_create_slave(bus, "ssi-sd");
		cs_line = qdev_get_gpio_in_named(dev1, "ssi-gpio-cs",  0);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1 , cs_line);
	}



	{
		DeviceState *dev;
		SysBusDevice *s;
		dev = qdev_create(NULL, "ls1a_dma");
		printf("ac97 dev=%p\n",dev);
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x1fd01160);
	}

#if 1
	{
		DeviceState *dev;
		SysBusDevice *s;
		dev = qdev_create(NULL, "ls1a_nand");
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x1fe78000);
		sysbus_connect_irq(s, 0, env->irq[6]);
	}
#endif


	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fe78030, "0x1fe78030", 0x8);
                memory_region_add_subregion(address_space_mem, 0x1fe78030, iomem);
	}

	{

                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fd00420, "0x1fd00420", 0x8);
                memory_region_add_subregion(address_space_mem, 0x1fd00420, iomem);
	}

	{
                ddrcfg_iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(ddrcfg_iomem, NULL, &mips_qemu_ops, (void *)(0x0ffffe00&TARGET_PAGE_MASK), "ddr", TARGET_PAGE_SIZE);
		if(ddr2config)
		{
		mips_qemu_writel((void *)0x1fd00424, 0, 0, 4);
		}
	}

}


static void mips_machine_init(MachineClass *mc)
{
    mc->desc = "mips ls232 platform";
    mc->init = mips_ls232_init;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("LS232");
}

DEFINE_MACHINE("ls232", mips_machine_init)
