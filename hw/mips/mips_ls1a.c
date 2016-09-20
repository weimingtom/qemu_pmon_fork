/*
 * QEMU loongson 1a develop board emulation
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
#include "hw/ide/internal.h"

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
static int reg0420[1]={0x2};

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

			if(val&0x2 && !(reg0420[0]&2))
			{
				memory_region_del_subregion(get_system_memory(), ddrcfg_iomem);
			}

			if(!(val&0x2) && (reg0420[0]&0x2))
			{
				memory_region_add_subregion_overlap(get_system_memory(), 0x0ffffe00&TARGET_PAGE_MASK, ddrcfg_iomem, 1);
			}

			reg0420[0] = val;

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
		case 0x1fd00420:
			return reg0420[0];
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


static void *ls1a_intctl_init(MemoryRegion *mr, hwaddr addr, qemu_irq parent_irq);

static int board_map_irq(int bus,int dev,int func,int pin)
{
	int irq_num=pin;
	return irq_num;
}

static const int sector_len = 32 * 1024;

static PCIBus *pcibus_ls1a_init(qemu_irq *pic, int (*board_map_irq)(int bus,int dev,int func,int pin));

static CPUUnassignedAccess real_do_unassigned_access;
static void mips_ls1a_do_unassigned_access(CPUState *cpu, hwaddr addr,
                                           bool is_write, bool is_exec,
                                           int opaque, unsigned size)
{
    if (!is_exec) {
        /* ignore invalid access (ie do not raise exception) */
        return;
    }
    (*real_do_unassigned_access)(cpu, addr, is_write, is_exec, opaque, size);
}

static void mips_ls1a_init (MachineState *args)
{
	ram_addr_t ram_size = args->ram_size;
	const char *cpu_model = args->cpu_model;
	const char *kernel_filename = args->kernel_filename;
	const char *kernel_cmdline = args->kernel_cmdline;
	const char *initrd_filename = args->initrd_filename;
	char *filename;
	MemoryRegion *address_space_mem = get_system_memory();
	MemoryRegion *ram = g_new(MemoryRegion, 1);
	MemoryRegion *isa_io = g_new(MemoryRegion, 1);
	MemoryRegion *isa_mem = g_new(MemoryRegion, 1);
	MemoryRegion *bios;
	int bios_size;
	MIPSCPU *cpu;
	CPUMIPSState *env;
	ResetData *reset_info;
	qemu_irq *ls1a_irq,*ls1a_irq1;
	PCIBus *pci_bus;
	DriveInfo *flash_dinfo=NULL;
	int ddr2config = 0;
	//ISABus *isa_bus;
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
		cc->do_unassigned_access = mips_ls1a_do_unassigned_access;

		reset_info = g_malloc0(sizeof(ResetData));
		reset_info->cpu = cpu;
		reset_info->vector = env->active_tc.PC;
		qemu_register_reset(main_cpu_reset, reset_info);

		/* allocate RAM */
	memory_region_init_ram(ram, NULL, "mips_r4k.ram", ram_size, &error_fatal);
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
        memory_region_init_ram(bios, NULL, "mips_r4k.bios", BIOS_SIZE, &error_fatal);
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

	//isa_bus = 

    memory_region_init_alias(isa_io, NULL, "isa-io",
                             get_system_io(), 0, 0x00010000);
    memory_region_init(isa_mem, NULL, "isa-mem", 0x01000000);
    memory_region_add_subregion(get_system_memory(), 0x1c000000, isa_io);
    memory_region_add_subregion(get_system_memory(), 0x10000000, isa_mem);

    isa_bus_new(NULL, isa_mem, get_system_io(), &error_abort);


	ls1a_irq =ls1a_intctl_init(get_system_memory(), 0x1Fd01040, env->irq[2]);
	ls1a_irq1=ls1a_intctl_init(get_system_memory(), 0x1Fd01058, env->irq[3]);
	ls1a_intctl_init(get_system_memory(), 0x1Fd01070, env->irq[4]);
	ls1a_intctl_init(get_system_memory(), 0x1Fd01088, env->irq[5]);


	if (serial_hds[0])
		serial_mm_init(address_space_mem, 0x1fe40000, 0,ls1a_irq[2],115200,serial_hds[0], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[1])
		serial_mm_init(address_space_mem, 0x1fe44000, 0,ls1a_irq[3],115200,serial_hds[1], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[2])
		serial_mm_init(address_space_mem, 0x1fe48000, 0,ls1a_irq[4],115200,serial_hds[2], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[3])
		serial_mm_init(address_space_mem, 0x1fe4c000, 0,ls1a_irq[5],115200,serial_hds[3], DEVICE_NATIVE_ENDIAN);

	sysbus_create_simple("ls1a_fb", 0x1c301240, NULL);

	{
		MemoryRegion *i8042 = g_new(MemoryRegion, 1);
		i8042_mm_init(ls1a_irq[12], ls1a_irq[11], i8042, 0x10, 0x4);
		memory_region_add_subregion(address_space_mem, 0x1fe60000, i8042);
	}


	pci_bus=pcibus_ls1a_init(&ls1a_irq1[6],board_map_irq);


	sysbus_create_simple("exynos4210-ehci-usb",0x1fe00000, ls1a_irq1[0]);
	{
		DeviceState *dev;
		dev = qdev_create(NULL, "sysbus-ohci");
		qdev_prop_set_uint32(dev, "num-ports", 4);
		qdev_prop_set_uint64(dev, "dma-offset", 0);
		qdev_init_nofail(dev);
		sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1fe08000);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls1a_irq1[1]);

		//dev = sysbus_create_simple("sysbus-ohci", 0x1fe08000, ls1a_irq1[1]);
	}
	{
		DeviceState *dev;
		BusState *idebus[4];
		DriveInfo *hd;
		dev = qdev_create(NULL, "sysbus-ahci");
		qdev_prop_set_uint32(dev, "num-ports", 2);
		qdev_init_nofail(dev);
		sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1fe30000);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls1a_irq1[4]);
		hd=drive_get_next(IF_IDE);
		if(hd)
		{
			idebus[0] = qdev_get_child_bus(dev, "ide.0");

			dev = qdev_create(idebus[0], hd->media_cd ? "ide-cd" : "ide-hd");
			qdev_prop_set_uint32(dev, "unit", 0);
			qdev_prop_set_drive(dev, "drive", blk_by_legacy_dinfo(hd),&error_fatal);
			qdev_init_nofail(dev);
		}
	}
	sysbus_create_simple("ls1a_acpi",0x1fe7c000, ls1a_irq[0]);

	if (nb_nics) {
		gmac_sysbus_create(&nd_table[0], 0x1fe10000, ls1a_irq1[2]);
	   if(nb_nics>1)
	  {
		PCIDevice *dev;
            dev = pci_nic_init_nofail(&nd_table[1], pci_bus, nd_table[1].model?:"e1000","0b");
	    printf("nb_nics=%d dev=%p\n", nb_nics, dev);
	  }
	}

#if 1
	{
		sysbus_create_simple("ls1a_wdt", 0x1fe7c060, NULL);
	}
#endif

	{
		DeviceState *dev,*dev1;
		void *bus;
		qemu_irq cs_line;
		dev=sysbus_create_simple("ls1a_spi",0x1fe80000, ls1a_irq[8]);
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
		cs_line = qdev_get_gpio_in(dev1, 0);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1 , cs_line);
	}


	{
	int i;
		DriveInfo *hd[MAX_IDE_BUS * MAX_IDE_DEVS];
		for(i = 0; i < MAX_IDE_BUS * MAX_IDE_DEVS; i++) {
			hd[i] = drive_get_next(IF_IDE);
		}
		pci_cmd646_ide_init(pci_bus, hd, 0);
	}

	{
		DeviceState *dev;
		SysBusDevice *s;
		dev = qdev_create(NULL, "ls1a_ac97");
		printf("ac97 dev=%p\n",dev);
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x1fe74000);
		sysbus_connect_irq(s, 0, ls1a_irq[14]);
		sysbus_connect_irq(s, 1, ls1a_irq[15]);
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
		sysbus_connect_irq(s, 0, ls1a_irq[13]);
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
		sysbus_connect_irq(s, 0, ls1a_irq[26]);
		sysbus_connect_irq(s, 1, ls1a_irq[27]);
		sysbus_connect_irq(s, 2, ls1a_irq[28]);
	}
#endif

	{
		DeviceState *dev;
		void *bus;
		dev=sysbus_create_simple("ls1a_i2c",0x1fe58000, ls1a_irq[17]);
		bus = qdev_get_child_bus(dev, "i2c");
		i2c_create_slave(bus, "ds1338", 0x68);
	}

#if 1
	{
		sysbus_create_simple("ls1a_can",0x1fe50000, ls1a_irq[6]);
		sysbus_create_simple("ls1a_can",0x1fe54000, ls1a_irq[7]);
	}
#endif

	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fe78030, "0x1fe78030", 0x8);
                memory_region_add_subregion(address_space_mem, 0x1fe78030, iomem);
	}

	{

                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fd00420, "0x1fd00420", 0x4);
                memory_region_add_subregion(address_space_mem, 0x1fd00420, iomem);
	}

	{
                ddrcfg_iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(ddrcfg_iomem, NULL, &mips_qemu_ops, (void *)(0x0ffffe00&TARGET_PAGE_MASK), "ddr", TARGET_PAGE_SIZE);
		if(ddr2config)
		{
		mips_qemu_writel((void *)0x1fd00420, 0, 0x0, 4);
		}
	}

}


static void mips_machine_init(MachineClass *mc)
{
    mc->desc = "mips ls1a platform";
    mc->init = mips_ls1a_init;
}

DEFINE_MACHINE("ls1a", mips_machine_init)

//------------
#include "hw/pci/pci.h"
#include "hw/pci/pci_host.h"


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

#include "ls1a_int.c"
//-------------------------
// pci bridge
//-----------------
static int (*local_board_map_irq)(int bus,int dev,int func,int pin);

static int pci_ls1a_map_irq(PCIDevice *d, int irq_num)
{
	int dev=(d->devfn>>3)&0x1f;
	int func=d->devfn& 7;

	return local_board_map_irq(0,dev,func,irq_num);
}

static void pci_ls1a_set_irq(void *opaque, int irq_num, int level)
{
qemu_irq *pic = opaque;
	qemu_set_irq(pic[irq_num],level);
}

#define PCI_LOCAL_REG_ADDRESS 0x1fd01114

#define TYPE_BONITO_PCI_HOST_BRIDGE "ls1a-pcihost"
typedef struct BonitoState BonitoState;

#define BONITO_PCI_HOST_BRIDGE(obj) \
    OBJECT_CHECK(BonitoState, (obj), TYPE_BONITO_PCI_HOST_BRIDGE)

typedef struct PCIBonitoState
{
	PCIDevice dev;
	BonitoState *pcihost;
	MemoryRegion iomem;
	struct pcilocalreg{
		unsigned int pcimap;
		unsigned int pcix_rgagte;
		unsigned int pcix_noused;
		unsigned int pcimap_cfg;
	} mypcilocalreg;
} PCIBonitoState;

struct BonitoState {
    PCIHostState parent_obj;
    qemu_irq *pic;
    PCIBonitoState *pci_dev;
};

static inline uint32_t bonito_pci_config_addr(PCIBonitoState *s,hwaddr addr)
{
	int bus = 0, dev = -1, func = 0, reg = 0;
	uint32_t busaddr;
	struct pcilocalreg *d=&s->mypcilocalreg;
	busaddr = ((d->pcimap_cfg & 0xffff) << 16) | (addr & 0xfffc);

	if (d->pcimap_cfg & 0x10000) {
	bus = busaddr >> 16;
	dev = (busaddr >> 11) & 0x1f;
	func = (busaddr >> 8) & 7;
	reg = busaddr & 0xfc;
	}
	else {
		bus = 0;
		dev = ffs(busaddr>>11)-1;
		func = (busaddr >> 8) & 7;
		reg = busaddr & 0xfc;
	}

	return bus<<16|dev<<11|func<<8|reg;
}

static void pci_ls1a_config_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	PCIBonitoState *s = opaque;
	//   PCIDevice *d = PCI_DEVICE(s);
	PCIHostState *phb = PCI_HOST_BRIDGE(s->pcihost);
	pci_data_write(phb->bus, bonito_pci_config_addr(s, addr), val, size);
}

static uint64_t pci_ls1a_config_readl (void *opaque, hwaddr addr, unsigned size)
{
	PCIBonitoState *s = opaque;
	//  PCIDevice *d = PCI_DEVICE(s);
	PCIHostState *phb = PCI_HOST_BRIDGE(s->pcihost);
	uint32_t val;
	val = pci_data_read(phb->bus, bonito_pci_config_addr(s, addr), size);
	return val;
}


static const MemoryRegionOps pci_ls1a_config_ops = {
    .read = pci_ls1a_config_readl,
    .write = pci_ls1a_config_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static uint64_t pci_bonito_local_readl (void *opaque, hwaddr addr, unsigned size)
{
    PCIBonitoState *s = opaque;
	uint32_t val;
	uint32_t relative_addr=addr;
	val = ((uint32_t *)&s->mypcilocalreg)[relative_addr/sizeof(uint32_t)];
	switch(size)
	{
	case 1:
	val = (val>>(addr&3))&0xff;
	break;
	case 2:
	val = (val>>(addr&3))&0xffff;
	break;
	}
	return val;
}

static void pci_bonito_local_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
    PCIBonitoState *s = opaque;
	uint32_t relative_addr=addr;
	((uint32_t *)&s->mypcilocalreg)[relative_addr/sizeof(uint32_t)]=val;
}

static const MemoryRegionOps pci_bonito_local_ops = {
    .read = pci_bonito_local_readl,
    .write = pci_bonito_local_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void bonito_pciconf_writel(void *opaque, hwaddr addr,
                                  uint64_t val, unsigned size)
{
    PCIBonitoState *s = opaque;
    PCIDevice *d = PCI_DEVICE(s);

    d->config_write(d, addr, val, 4);
}

static uint64_t bonito_pciconf_readl(void *opaque, hwaddr addr,
                                     unsigned size)
{

    PCIBonitoState *s = opaque;
    PCIDevice *d = PCI_DEVICE(s);

    return d->config_read(d, addr, 4);
}

/* north bridge PCI configure space. 0x1fe0 0000 - 0x1fe0 00ff */

static const MemoryRegionOps bonito_pciconf_ops = {
    .read = bonito_pciconf_readl,
    .write = bonito_pciconf_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static int bonito_initfn(PCIDevice *dev)
{
    PCIBonitoState *s = DO_UPCAST(PCIBonitoState, dev, dev);
    SysBusDevice *sysbus = SYS_BUS_DEVICE(s->pcihost);
    PCIHostState *phb = PCI_HOST_BRIDGE(s->pcihost);

    /* Bonito North Bridge, built on FPGA, VENDOR_ID/DEVICE_ID are "undefined" */
    pci_config_set_prog_interface(dev->config, 0x00);


    memory_region_init_io(&s->iomem, NULL, &pci_bonito_local_ops, s, "ls1a_pci_conf", 16);
    sysbus_init_mmio(sysbus, &s->iomem);
    sysbus_mmio_map(sysbus, 0, PCI_LOCAL_REG_ADDRESS);

    /* set the north bridge pci configure  mapping */
    memory_region_init_io(&phb->conf_mem, NULL, &bonito_pciconf_ops, s,
                          "north-bridge-pci-config", 0x100);
    sysbus_init_mmio(sysbus, &phb->conf_mem);
    sysbus_mmio_map(sysbus, 1, 0x1c110000);

    /* set the south bridge pci configure  mapping */
    memory_region_init_io(&phb->data_mem, NULL, &pci_ls1a_config_ops, s,
                          "south-bridge-pci-config", 0x10000);
    sysbus_init_mmio(sysbus, &phb->data_mem);
    sysbus_mmio_map(sysbus, 2, 0x1c100000);

    /* set the default value of north bridge pci config */
    pci_set_word(dev->config + PCI_COMMAND, 0x0000);
    pci_set_word(dev->config + PCI_STATUS, 0x0000);
    pci_set_word(dev->config + PCI_SUBSYSTEM_VENDOR_ID, 0x0000);
    pci_set_word(dev->config + PCI_SUBSYSTEM_ID, 0x0000);

    pci_set_byte(dev->config + PCI_INTERRUPT_LINE, 0x00);
    pci_set_byte(dev->config + PCI_INTERRUPT_PIN, 0x01);
    pci_set_byte(dev->config + PCI_MIN_GNT, 0x3c);
    pci_set_byte(dev->config + PCI_MAX_LAT, 0x00);


    return 0;
}


static void bonito_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->init = bonito_initfn;
    k->vendor_id = 0xdf53;
    k->device_id = 0x00d5;
    k->revision = 0x01;
    k->class_id = PCI_CLASS_BRIDGE_HOST;
    dc->desc = "Host bridge";
}

static const TypeInfo bonito_info = {
    .name          = "LS1A_Bonito",
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PCIBonitoState),
    .class_init    = bonito_class_init,
};


static PCIBus *pcibus_ls1a_init(qemu_irq *pic, int (*board_map_irq)(int bus,int dev,int func,int pin))
{
    DeviceState *dev;
    BonitoState *pcihost;
    PCIHostState *phb;
    PCIBonitoState *s;
    PCIDevice *d;
	local_board_map_irq = board_map_irq;

    dev = qdev_create(NULL, TYPE_BONITO_PCI_HOST_BRIDGE);
    phb = PCI_HOST_BRIDGE(dev);
    pcihost = BONITO_PCI_HOST_BRIDGE(dev);
    pcihost->pic = pic;
    qdev_init_nofail(dev);

    /* set the pcihost pointer before bonito_initfn is called */
    d = pci_create(phb->bus, PCI_DEVFN(0, 0), "LS1A_Bonito");
    s = DO_UPCAST(PCIBonitoState, dev, d);
    s->pcihost = pcihost;
    pcihost->pci_dev = s;
    qdev_init_nofail(DEVICE(d));


    return phb->bus;
}

static int bonito_pcihost_initfn(SysBusDevice *dev)
{
    BonitoState *pcihost;
    PCIHostState *phb = PCI_HOST_BRIDGE(dev);
    pcihost = BONITO_PCI_HOST_BRIDGE(dev);

    phb->bus = pci_register_bus(DEVICE(dev), "pci",
                                pci_ls1a_set_irq, pci_ls1a_map_irq, pcihost->pic,
                                get_system_memory(), get_system_io(),
                                1<<3, 4, TYPE_PCI_BUS);

    return 0;
}

static void bonito_pcihost_class_init(ObjectClass *klass, void *data)
{
    //DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = bonito_pcihost_initfn;
}

static const TypeInfo bonito_pcihost_info = {
    .name          = TYPE_BONITO_PCI_HOST_BRIDGE,
    .parent        = TYPE_PCI_HOST_BRIDGE,
    .instance_size = sizeof(BonitoState),
    .class_init    = bonito_pcihost_class_init,
};

static void bonito_register_types(void)
{
    type_register_static(&bonito_pcihost_info);
    type_register_static(&bonito_info);
}

type_init(bonito_register_types)
