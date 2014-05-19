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
#include "hw/ssi.h"
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
			(uint64_t *)&kernel_high,0,ELF_MACHINE, 1);
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


void *ls1b_intctl_init(hwaddr addr,qemu_irq parent_irq);

static const int sector_len = 32 * 1024;

static CPUUnassignedAccess real_do_unassigned_access;
static void mips_ls1b_do_unassigned_access(CPUState *cpu, hwaddr addr,
                                           bool is_write, bool is_exec,
                                           int opaque, unsigned size)
{
    if (!is_exec) {
        /* ignore invalid access (ie do not raise exception) */
        return;
    }
    (*real_do_unassigned_access)(cpu, addr, is_write, is_exec, opaque, size);
}

static void mips_ls1b_init (QEMUMachineInitArgs *args)
{
	ram_addr_t ram_size = args->ram_size;
	const char *cpu_model = args->cpu_model;
	const char *kernel_filename = args->kernel_filename;
	const char *kernel_cmdline = args->kernel_cmdline;
	const char *initrd_filename = args->initrd_filename;
	char *filename;
	MemoryRegion *address_space_mem = get_system_memory();
	MemoryRegion *ram = g_new(MemoryRegion, 1);
	MemoryRegion *isa = g_new(MemoryRegion, 1);
	MemoryRegion *bios;
	int bios_size;
	MIPSCPU *cpu;
	CPUMIPSState *env;
	ResetData *reset_info;
	qemu_irq *ls1b_irq,*ls1b_irq1;
	DriveInfo *flash_dinfo=NULL;
	int ddr2config = 0;
	CPUClass *cc;


	/* init CPUs */
	if (cpu_model == NULL) {
#ifdef TARGET_MIPS64
		cpu_model = "LS232";
#else
		cpu_model = "LS232";
#endif
	}

		cpu = cpu_mips_init(cpu_model);
		if (cpu == NULL) {
			fprintf(stderr, "Unable to find CPU definition\n");
			exit(1);
		}
		env = &cpu->env;

		cc = CPU_GET_CLASS(cpu);
		real_do_unassigned_access = cc->do_unassigned_access;
		cc->do_unassigned_access = mips_ls1b_do_unassigned_access;

		reset_info = g_malloc0(sizeof(ResetData));
		reset_info->cpu = cpu;
		reset_info->vector = env->active_tc.PC;
		qemu_register_reset(main_cpu_reset, reset_info);

		/* allocate RAM */
	memory_region_init_ram(ram, NULL, "mips_r4k.ram", ram_size);
	vmstate_register_ram_global(ram);

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
        memory_region_init_ram(bios, NULL, "mips_r4k.bios", BIOS_SIZE);
        vmstate_register_ram_global(bios);
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
	cpu_mips_irq_init_cpu(env);
	cpu_mips_clock_init(env);

	/* Register 64 KB of IO space at 0x1f000000 */
	isa_bus_new(NULL, get_system_io());

	/* ISA IO space at 0x90000000 */
	memory_region_init_alias(isa, NULL, "isa_mmio",
			get_system_io(), 0, 0x010000);
	memory_region_add_subregion(get_system_memory(), 0x1c000000, isa);
	isa_mem_base = 0x10000000;

	ls1b_irq =ls1b_intctl_init(0x1Fd01040,env->irq[2]);
	ls1b_irq1=ls1b_intctl_init(0x1Fd01058,env->irq[3]);
	ls1b_intctl_init(0x1Fd01070,env->irq[4]);
	ls1b_intctl_init(0x1Fd01088,env->irq[5]);


	if (serial_hds[0])
		serial_mm_init(address_space_mem, 0x1fe40000, 0,ls1b_irq[2],115200,serial_hds[0], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[1])
		serial_mm_init(address_space_mem, 0x1fe44000, 0,ls1b_irq[3],115200,serial_hds[1], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[2])
		serial_mm_init(address_space_mem, 0x1fe48000, 0,ls1b_irq[4],115200,serial_hds[2], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[3])
		serial_mm_init(address_space_mem, 0x1fe4c000, 0,ls1b_irq[5],115200,serial_hds[3], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[4])
		serial_mm_init(address_space_mem, 0x1fe6c000, 0,ls1b_irq[29],115200,serial_hds[4], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[5])
		serial_mm_init(address_space_mem, 0x1fe7c000, 0,ls1b_irq[30],115200,serial_hds[5], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[6])
		serial_mm_init(address_space_mem, 0x1fe41000, 0,ls1b_irq[2],115200,serial_hds[6], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[7])
		serial_mm_init(address_space_mem, 0x1fe42000, 0,ls1b_irq[2],115200,serial_hds[7], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[8])
		serial_mm_init(address_space_mem, 0x1fe43000, 0,ls1b_irq[2],115200,serial_hds[8], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[9])
		serial_mm_init(address_space_mem, 0x1fe45000, 0,ls1b_irq[3],115200,serial_hds[9], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[10])
		serial_mm_init(address_space_mem, 0x1fe46000, 0,ls1b_irq[3],115200,serial_hds[10], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[11])
		serial_mm_init(address_space_mem, 0x1fe47000, 0,ls1b_irq[3],115200,serial_hds[11], DEVICE_NATIVE_ENDIAN);

	sysbus_create_simple("ls1a_fb", 0x1c301240, NULL);

	{
		MemoryRegion *i8042 = g_new(MemoryRegion, 1);
		i8042_mm_init(ls1b_irq[12], ls1b_irq[11], i8042, 0x10, 0x4);
		memory_region_add_subregion(address_space_mem, 0x1fe60000, i8042);
	}


//	sysbus_create_simple("exynos4210-ehci-usb",0x1fe00000, ls1b_irq1[0]);
	{
		DeviceState *dev;
		dev = qdev_create(NULL, "sysbus-ohci");
		qdev_prop_set_uint32(dev, "num-ports", 4);
		qdev_prop_set_uint64(dev, "dma-offset", 0);
		qdev_init_nofail(dev);
		sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1fe08000);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls1b_irq1[1]);

		//dev = sysbus_create_simple("sysbus-ohci", 0x1fe08000, ls1b_irq1[1]);
	}
	sysbus_create_simple("sysbus-ahci",0x1fe30000, ls1b_irq1[4]);

	if (nb_nics) {
		gmac_sysbus_create(&nd_table[0], 0x1fe10000, ls1b_irq1[2]);
	}

#if 1
	{
		sysbus_create_simple("ls1a_wdt", 0x1fe5c060, NULL);
	}
#endif

	{
		DeviceState *dev,*dev1;
		void *bus;
		qemu_irq cs_line;
		dev=sysbus_create_simple("ls1a_spi",0x1fe80000, ls1b_irq[8]);
		bus = qdev_get_child_bus(dev, "ssi");
		if(flash_dinfo)
		{
			dev1 = ssi_create_slave_no_init(bus, "spi-flash");
			if (qdev_prop_set_drive(dev1, "drive", flash_dinfo->bdrv)) {
				abort();
			}
			qdev_prop_set_uint32(dev1, "size", 0x100000);
			qdev_prop_set_uint64(dev1, "addr", 0x1fc00000);
			qdev_init_nofail(dev1);
		}
		else dev1 = ssi_create_slave(bus, "ssi-sd");
		cs_line = qdev_get_gpio_in(dev1, 0);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1 , cs_line);
	}


	{
		DeviceState *dev;
		SysBusDevice *s;
		dev = qdev_create(NULL, "ls1a_ac97");
		printf("ac97 dev=%p\n",dev);
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x1fe74000);
		sysbus_connect_irq(s, 0, ls1b_irq[14]);
		sysbus_connect_irq(s, 1, ls1b_irq[15]);
	}

	{
		DeviceState *dev;
		SysBusDevice *s;
		dev = qdev_create(NULL, "ls1b_dma");
		printf("ac97 dev=%p\n",dev);
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x1fd01160);
	}

#if 1
	{
		DeviceState *dev;
		SysBusDevice *s;
		dev = qdev_create(NULL, "ls1b_nand");
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x1fe78000);
		sysbus_connect_irq(s, 0, ls1b_irq[13]);
	}
#endif

#if 1
	{
		DeviceState *dev;
		SysBusDevice *s;
		dev = qdev_create(NULL, "ls1a_rtc");
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x1fe64020);
		sysbus_connect_irq(s, 0, ls1b_irq[26]);
		sysbus_connect_irq(s, 1, ls1b_irq[27]);
		sysbus_connect_irq(s, 2, ls1b_irq[28]);
	}
#endif

	{
		DeviceState *dev;
		void *bus;
		dev=sysbus_create_simple("ls1a_i2c",0x1fe58000, ls1b_irq[17]);
		bus = qdev_get_child_bus(dev, "i2c");
		i2c_create_slave(bus, "ds1338", 0x68);
	}

#if 1
	{
		sysbus_create_simple("ls1a_can",0x1fe50000, ls1b_irq[6]);
		sysbus_create_simple("ls1a_can",0x1fe54000, ls1b_irq[7]);
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

QEMUMachine mips_ls1b_machine = {
	.name = "ls1b",
	.desc = "mips ls1b platform",
	.init = mips_ls1b_init,
};

static void mips_machine_init(void)
{
	qemu_register_machine(&mips_ls1b_machine);
}

machine_init(mips_machine_init);

//------------
//#define DEBUG_IRQ

#ifdef DEBUG_IRQ
#define DPRINTF(fmt, args...) \
	do { printf("IRQ: " fmt , ##args); } while (0)
#else
#define DPRINTF(fmt, args...)
#endif

/*
 * Registers of interrupt controller in sun4m.
 *
 * This is the interrupt controller part of chip STP2001 (Slave I/O), also
 * produced as NCR89C105. See
 * http://www.ibiblio.org/pub/historic-linux/early-ports/Sparc/NCR/NCR89C105.txt
 *
 * There is a system master controller and one for each cpu.
 *
 */

#define MAX_CPUS 1
#define MAX_PILS 16

typedef struct GS232_INTCTLState {
	uint32_t baseaddr;
	uint32_t intreg_edge;
	uint32_t intreg_steer;
	uint32_t intreg_pol;
	//set
	//clr
	uint32_t intreg_en;
	uint32_t intreg_pending;
#ifdef DEBUG_IRQ_COUNT
	uint64_t irq_count[32];
#endif
	qemu_irq cpu_irq;
	uint32_t pil_out;
} GS232_INTCTLState;

#define INTCTL_SIZE 0x18
#define INTCTLM_MAXADDR 0x13
#define INTCTLM_SIZE (INTCTLM_MAXADDR + 1)
#define INTCTLM_MASK 0x1f
#define MASTER_IRQ_MASK ~0x0fa2007f
#define MASTER_DISABLE 0x80000000
#define CPU_SOFTIRQ_MASK 0xfffe0000
#define CPU_HARDIRQ_MASK 0x0000fffe
#define CPU_IRQ_INT15_IN 0x0004000
#define CPU_IRQ_INT15_MASK 0x80000000

static void ls1b_check_interrupts(void *opaque);

// per-cpu interrupt controller
static uint64_t ls1b_intctl_mem_readl(void *opaque, hwaddr addr, unsigned size)
{
	GS232_INTCTLState *s = opaque;
	uint32_t saddr, ret;

	saddr = addr >> 2;
	switch (saddr) {
		case 0: //isr
			ret = s->intreg_pending & s->intreg_en;
			break;
		case 1:
			ret= s->intreg_en;
			break;
		case 2: //set
			ret=0;
			break;
		case 3: //clr
			ret=0;
			break;
		case 4:
			ret= s->intreg_pol;
			break;
		case 5:
			ret= s->intreg_edge;
			break;
		default:
			ret = 0;
			break;
	}
	DPRINTF("read cpu %d reg 0x" TARGET_FMT_plx " = %x\n", cpu, addr, ret);

	return ret;
}

static void ls1b_intctl_mem_writel(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
	GS232_INTCTLState *s = opaque;
	uint32_t saddr;

	saddr = addr >> 2;
	//printf("write reg 0x" TARGET_FMT_plx " %x= %x\n", addr, saddr, (unsigned int)val);
	switch (saddr) {
		case 0: //isr
			s->intreg_pending=val;
			ls1b_check_interrupts(s);
			break;
		case 1:
			s->intreg_en=val;
			ls1b_check_interrupts(s);
			break;
		case 2: //set
			s->intreg_en |= val;
			ls1b_check_interrupts(s);
			break;
		case 3: //clr
			s->intreg_pending &= ~(val & s->intreg_edge);
			ls1b_check_interrupts(s);
			break;
		case 4:
			s->intreg_pol=val;
			ls1b_check_interrupts(s);
			break;
		case 5:
			s->intreg_edge=val;
			ls1b_check_interrupts(s);
			break;
		default:
			break;
	}
}

static const MemoryRegionOps ls1b_intctl_mem_ops = {
    .read = ls1b_intctl_mem_readl,
    .write = ls1b_intctl_mem_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static void ls1b_check_interrupts(void *opaque)
{
	GS232_INTCTLState *s = opaque;
	uint32_t pil_pending;


	pil_pending = s->intreg_pending & s->intreg_en;

	if (pil_pending ) {
		qemu_irq_raise(s->cpu_irq);
	} else {
		if (s->pil_out)
			qemu_irq_lower(s->cpu_irq);
	}
	s->pil_out = pil_pending;
	DPRINTF("pending %x \n", pil_pending);
}

/*
 * "irq" here is the bit number in the system interrupt register to
 * separate serial and keyboard interrupts sharing a level.
 */
static void ls1b_set_irq(void *opaque, int irq, int level)
{
	GS232_INTCTLState *s = opaque;
	uint32_t mask = 1 << irq;

	DPRINTF("Set cpu %d irq %d level %d\n", s->target_cpu, irq,
			level);
	if (level) {
		s->intreg_pending |= mask;
	} else {
		s->intreg_pending &= ~mask | s->intreg_edge;
	}
	ls1b_check_interrupts(s);
}


static void ls1b_intctl_save(QEMUFile *f, void *opaque)
{
	GS232_INTCTLState *s = opaque;

	qemu_put_be32s(f, &s->intreg_pending);
}

static int ls1b_intctl_load(QEMUFile *f, void *opaque, int version_id)
{
	GS232_INTCTLState *s = opaque;

	if (version_id != 1)
		return -EINVAL;

	qemu_get_be32s(f, &s->intreg_pending);
	ls1b_check_interrupts(s);
	return 0;
}

static void ls1b_intctl_reset(void *opaque)
{
	GS232_INTCTLState *s = opaque;

	s->intreg_pending = 0;
	ls1b_check_interrupts(s);
}


void *ls1b_intctl_init(hwaddr addr,qemu_irq parent_irq)
{
	qemu_irq *irqs;
	GS232_INTCTLState *s;

	s = g_malloc0(sizeof(GS232_INTCTLState));
	if (!s)
		return NULL;

	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &ls1b_intctl_mem_ops, s, "ls1b_int", INTCTL_SIZE);
                memory_region_add_subregion(get_system_memory(), addr, iomem);
	}

	s->cpu_irq = parent_irq;
	s->baseaddr=addr;

	register_savevm(NULL, "ls1b_intctl", addr, 1, ls1b_intctl_save, ls1b_intctl_load, s);
	qemu_register_reset(ls1b_intctl_reset, s);
	irqs = qemu_allocate_irqs(ls1b_set_irq, s, 32);

	ls1b_intctl_reset(s);
	return irqs;
}

