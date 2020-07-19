/*
 * QEMU loongson2f sm502 develop board emulation
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
#include "hw/timer/mc146818rtc.h"
#include "hw/timer/i8254.h"
#include "sysemu/blockdev.h"
#include "exec/address-spaces.h"

#define PHYS_TO_VIRT(x) ((x) | ~(target_ulong)0x7fffffff)
#define MAX_IDE_BUS 2

#define TARGET_REALPAGE_MASK (TARGET_PAGE_MASK<<2)

static const int ide_iobase[2] = { 0x1f0, 0x170 };
static const int ide_iobase2[2] = { 0x3f6, 0x376 };
static const int ide_irq[2] = { 14, 15 };

/* i8254 PIT is attached to the IRQ0 at PIC i8259 */

static struct _loaderparams {
    uint64_t ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
} loaderparams;



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
	    if(getenv("INITRD_OFFSET")) initrd_offset = strtoul(getenv("INITRD_OFFSET"), 0, 0);
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
#define PBUF_SIZE 1024
	params_size = PBUF_SIZE + 8;
	params_buf = g_malloc(params_size);

	

#define BOOTPARAM_PHYADDR ((64 << 20) - PBUF_SIZE - 8)
#define BOOTPARAM_ADDR (0x80000000+BOOTPARAM_PHYADDR)
// should set argc,argv
//env->gpr[REG][env->current_tc]
	{
		char memenv[32];
		char highmemenv[32];
		const char *pmonenv[]={"cpuclock=200000000",memenv,highmemenv};
		int i;
		unsigned int *parg_env=(void *)params_buf;
		/*
		 * pram buf like this:
		 *argv[0] argv[1] 0 env[0] env[1] ...env[i] ,0, argv[0]'s data , argv[1]'s data ,env[0]'data,...,env[i]'s dat,0
		 */

		//*count user special env
		for(ret=0,i=0;environ[i];i++)
			if(!strncmp(environ[i],"ENV_",4))ret+=4;

		//jump over argv and env area
		ret +=(3+sizeof(pmonenv)/sizeof(char *)+1)*4;
		//argv0
		*parg_env++=BOOTPARAM_ADDR+ret;
		ret +=1+snprintf(params_buf+ret,PBUF_SIZE-ret,"g");
		//argv1
		*parg_env++=BOOTPARAM_ADDR+ret;
		if (initrd_size > 0) {
			ret +=1+snprintf(params_buf+ret,PBUF_SIZE-ret, "rd_start=0x" TARGET_FMT_lx " rd_size=%li %s",
					PHYS_TO_VIRT((uint32_t)initrd_offset),
					initrd_size, loaderparams.kernel_cmdline);
		} else {
			ret +=1+snprintf(params_buf+ret, PBUF_SIZE-ret, "%s", loaderparams.kernel_cmdline);
		}
		//argv2
		*parg_env++=0;

		//env
		sprintf(memenv,"memsize=%d",loaderparams.ram_size>0x10000000?256:(loaderparams.ram_size>>20));
		sprintf(highmemenv,"highmemsize=%d",loaderparams.ram_size>0x10000000?(loaderparams.ram_size>>20)-256:0);


		for(i=0;i<sizeof(pmonenv)/sizeof(char *);i++)
		{
			*parg_env++=BOOTPARAM_ADDR+ret;
			ret +=1+snprintf(params_buf+ret,PBUF_SIZE-ret,"%s",pmonenv[i]);
		}

		for(i=0;environ[i];i++)
		{
			if(!strncmp(environ[i],"ENV_",4)){
				*parg_env++=BOOTPARAM_ADDR+ret;
				ret +=1+snprintf(params_buf+ret,PBUF_SIZE-ret,"%s",&environ[i][4]);
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
	env->active_tc.gpr[5]=(target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR;
	env->active_tc.gpr[6]=(target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR +12;
}



static int board_map_irq_sm502(int bus,int dev,int func,int pin)
{
int irq_num=pin;
	if(dev>=14 && dev<=16)
     irq_num=((dev-14)+pin)%4;

    return irq_num;
}

static int aui_boot_code[] ={
0x04110002,/*   bal 1f*/
0x00000000,/*   nop*/
0x00000000,/*   .word 0*/
0x8FFF0000,/*   lw $ra,($ra)*/
0x3C08BFE0,/*   li $t0,0xbfe00118*/
0x35080118,/* */
0x24090200,/*   li $t1,((1 << (14 + 11)) | (0 << 8) | 14)>>16*/
0xAD090000,/*   sw $t1, ($t0)*/
0x3C08BFE8,/*   li $t0,0xbfe80000+(((1 << (14 + 11)) | (0 << 8) | 0)&0xffff)*/
0x3C091500,/*   li $t1,0x15000000*/
0xAD090014,/*   sw $t1,0x14($t0)*/
0x24090007,/*   li $t1,  7*/
0xAD090004,/*   sw $t1, 4($t0)*/
0x03E00008,/*   jr $ra*/
0x00000000,/*   nop*/
0x00000000,/* */
};

PCIBus *pci_bonito_init(CPUMIPSState *env,qemu_irq *pic, int irq,int (*board_map_irq)(int bus,int dev,int func,int pin),MemoryRegion *ram);
int pci_sm502_init(PCIBus *bus, int devfn, CharDriverState *chr);
static const int sector_len = 32 * 1024;
static void mips_ls2f_sm502_init (QEMUMachineInitArgs *args)
{
	ram_addr_t ram_size = args->ram_size;
	const char *cpu_model = args->cpu_model;
	const char *kernel_filename = args->kernel_filename;
	const char *kernel_cmdline = args->kernel_cmdline;
	const char *initrd_filename = args->initrd_filename;
	char *filename;
	MemoryRegion *address_space_mem = get_system_memory();
	MemoryRegion *ram = g_new(MemoryRegion, 1);
	MemoryRegion *bios;
	int bios_size;
	MIPSCPU *cpu;
	CPUMIPSState *env;
	ResetData *reset_info;
	int be;
	DriveInfo *dinfo;
	PCIBus *pci_bus;
	int i;


    /* init CPUs */
    if (cpu_model == NULL) {
#ifdef TARGET_MIPS64
        cpu_model = "Loongson-2F";
#else
        cpu_model = "Loongson-2F";
#endif
    }



		cpu = cpu_mips_init(cpu_model);
		if (cpu == NULL) {
			fprintf(stderr, "Unable to find CPU definition\n");
			exit(1);
		}
		env = &cpu->env;


    /* Init CPU internal devices */
    cpu_mips_irq_init_cpu(env);
    cpu_mips_clock_init(env);
		reset_info = g_malloc0(sizeof(ResetData));
		reset_info->cpu = cpu;
		reset_info->vector = env->active_tc.PC;
		qemu_register_reset(main_cpu_reset, reset_info);

		/* allocate RAM */
	memory_region_init_ram(ram, "mips_ls3a.ram", ram_size);

    /* allocate RAM */


	pci_bus=pci_bonito_init(env,(qemu_irq *)&env->irq[6],4,board_map_irq_sm502, ram);

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
#ifdef TARGET_WORDS_BIGENDIAN
    be = 1;
#else
    be = 0;
#endif
    if ((bios_size > 0) && (bios_size <= BIOS_SIZE)) {
        bios = g_new(MemoryRegion, 1);
        memory_region_init_ram(bios, NULL, "mips_r4k.bios", BIOS_SIZE);
        memory_region_set_readonly(bios, true);
        memory_region_add_subregion(get_system_memory(), 0x1fc00000, bios);

        load_image_targphys(filename, 0x1fc00000, BIOS_SIZE);
    } else if ((dinfo = drive_get(IF_PFLASH, 0, 0)) != NULL) {
        uint32_t mips_rom = 0x00400000;
        if (!pflash_cfi01_register(0x1fc00000, NULL, "mips_r4k.bios", mips_rom,
                                   dinfo->bdrv, sector_len,
                                   mips_rom / sector_len,
                                   4, 0, 0, 0, 0, be)) {
            fprintf(stderr, "qemu: Error registering flash memory.\n");
	}
    }


    if (filename) {
        g_free(filename);
    }

    if (kernel_filename) {
	    int bios_size = 0x1000;
        loaderparams.ram_size = ram_size;
		if(getenv("BD_PCI6254")) loaderparams.ram_size+=0x4000000;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        reset_info->vector = load_kernel();

        bios = g_new(MemoryRegion, 1);
        memory_region_init_ram(bios, NULL, "mips_r4k.bios", bios_size);
        memory_region_set_readonly(bios, true);
        memory_region_add_subregion(get_system_memory(), 0x1fc90000, bios);

	bios_size = sizeof(aui_boot_code);
	aui_boot_code[2] = reset_info->vector;
	rom_add_blob_fixed("bios",aui_boot_code,sizeof(aui_boot_code),0x1fc90000);
	reset_info->vector = 0xffffffffbfc90000LL;
    }





    /* Optional PCI video card */
	for(i=0;i<nb_nics;i++)
	{
		char devaddr[10];

		sprintf(devaddr,"%x",16+i);
		pci_nic_init(&nd_table[i], pci_bus, nd_table[i].model?:"rtl8139",devaddr);
	}


    if (serial_hds[0])
     pci_sm502_init(pci_bus,14<<3, serial_hds[0]);
    else 
     pci_sm502_init(pci_bus,14<<3, 0);

     pci_create_simple(pci_bus, -1, "pci-ohci");
	

    if (serial_hds[1])
        serial_mm_init(address_space_mem, getenv("NB_SERIAL")?strtoul(getenv("NB_SERIAL"),0,0):0x1fc803f8, 0,env->irq[2],115200, serial_hds[1],1);


}

QEMUMachine mips_gs2f_machine = {
    .name = "ls2f_sm502",
    .desc = "mips gs2f platform",
    .init = mips_ls2f_sm502_init,
};

static void mips_machine_init(void)
{
    qemu_register_machine(&mips_gs2f_machine);
}

machine_init(mips_machine_init);
