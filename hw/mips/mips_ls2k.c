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
#include "hw/i386/pc.h"
#include "hw/char/serial.h"
#include "hw/block/fdc.h"
#include "net/net.h"
#include "hw/boards.h"
#include "hw/i2c/smbus.h"
#include "block/block.h"
#include "hw/block/flash.h"
#include "hw/mips/mips.h"
#include "hw/mips/cpudevs.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_bridge.h"
#include "sysemu/char.h"
#include "sysemu/sysemu.h"
#include "sysemu/arch_init.h"
#include "qemu/log.h"
#include "hw/mips/bios.h"
#include "hw/ide.h"
#include "hw/loader.h"
#include "elf.h"
#include "hw/timer/mc146818rtc.h"
#include "hw/timer/i8254.h"
#include "sysemu/blockdev.h"
#include "exec/address-spaces.h"
#include "hw/sysbus.h"             /* SysBusDevice */
#include "qemu/host-utils.h"
#include "sysemu/qtest.h"
#include "qemu/error-report.h"
#include "hw/empty_slot.h"
#include "hw/ssi/ssi.h"
#include "loongson_bootparam.h"
#include <stdlib.h>

#define PHYS_TO_VIRT(x) ((x) | ~(target_ulong)0x7fffffff)

#define VIRT_TO_PHYS_ADDEND (-((int64_t)(int32_t)0x80000000))

#define MAX_IDE_BUS 2
#define TARGET_REALPAGE_MASK (TARGET_PAGE_MASK<<2)

static const int ide_iobase[2] = { 0x1f0, 0x170 };
static const int ide_iobase2[2] = { 0x3f6, 0x376 };
static const int ide_irq[2] = { 14, 15 };
static qemu_irq *ls2k_irq,*ls2k_irq1;

/* i8254 PIT is attached to the IRQ0 at PIC i8259 */

static struct _loaderparams {
    int ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
    target_ulong a0,a1,a2;
} loaderparams;

static void *boot_params_buf;
static void *boot_params_p;
#define align(x) (((x)+15)&~15)
static int pci_ls2k_map_irq(PCIDevice *d, int irq_num);

static struct {
unsigned long long runins;
unsigned long long pc_low;
unsigned long long pc_high;
unsigned long long skb;
unsigned long long skb_data;
unsigned long long stack;
unsigned int count;
} __attribute__((packed)) mynet ;

extern void (*mypc_callback)(target_ulong pc, uint32_t opcode);

typedef struct {
int pc;
unsigned long long addr;
} __attribute__((packed)) mypc;
static mypc *pcbuf;
static int pcbuf_size;
static int pcbuf_pos;

#define MASK_OP_MAJOR(op)  (op & (0x3F << 26))

enum {
    /* indirect opcode tables */
    OPC_SPECIAL  = (0x00 << 26),
    OPC_REGIMM   = (0x01 << 26),
    OPC_CP0      = (0x10 << 26),
    OPC_CP1      = (0x11 << 26),
    OPC_CP2      = (0x12 << 26),
    OPC_CP3      = (0x13 << 26),
    OPC_SPECIAL2 = (0x1C << 26),
    OPC_SPECIAL3 = (0x1F << 26),
    /* arithmetic with immediate */
    OPC_ADDI     = (0x08 << 26),
    OPC_ADDIU    = (0x09 << 26),
    OPC_SLTI     = (0x0A << 26),
    OPC_SLTIU    = (0x0B << 26),
    /* logic with immediate */
    OPC_ANDI     = (0x0C << 26),
    OPC_ORI      = (0x0D << 26),
    OPC_XORI     = (0x0E << 26),
    OPC_LUI      = (0x0F << 26),
    /* arithmetic with immediate */
    OPC_DADDI    = (0x18 << 26),
    OPC_DADDIU   = (0x19 << 26),
    /* Jump and branches */
    OPC_J        = (0x02 << 26),
    OPC_JAL      = (0x03 << 26),
    OPC_JALS     = OPC_JAL | 0x5,
    OPC_BEQ      = (0x04 << 26),  /* Unconditional if rs = rt = 0 (B) */
    OPC_BEQL     = (0x14 << 26),
    OPC_BNE      = (0x05 << 26),
    OPC_BNEL     = (0x15 << 26),
    OPC_BLEZ     = (0x06 << 26),
    OPC_BLEZL    = (0x16 << 26),
    OPC_BGTZ     = (0x07 << 26),
    OPC_BGTZL    = (0x17 << 26),
    OPC_JALX     = (0x1D << 26),  /* MIPS 16 only */
    OPC_JALXS    = OPC_JALX | 0x5,
    /* Load and stores */
    OPC_LDL      = (0x1A << 26),
    OPC_LDR      = (0x1B << 26),
    OPC_LB       = (0x20 << 26),
    OPC_LH       = (0x21 << 26),
    OPC_LWL      = (0x22 << 26),
    OPC_LW       = (0x23 << 26),
    OPC_LWPC     = OPC_LW | 0x5,
    OPC_LBU      = (0x24 << 26),
    OPC_LHU      = (0x25 << 26),
    OPC_LWR      = (0x26 << 26),
    OPC_LWU      = (0x27 << 26),
    OPC_SB       = (0x28 << 26),
    OPC_SH       = (0x29 << 26),
    OPC_SWL      = (0x2A << 26),
    OPC_SW       = (0x2B << 26),
    OPC_SDL      = (0x2C << 26),
    OPC_SDR      = (0x2D << 26),
    OPC_SWR      = (0x2E << 26),
    OPC_LL       = (0x30 << 26),
    OPC_LLD      = (0x34 << 26),
    OPC_LD       = (0x37 << 26),
    OPC_LDPC     = OPC_LD | 0x5,
    OPC_SC       = (0x38 << 26),
    OPC_SCD      = (0x3C << 26),
    OPC_SD       = (0x3F << 26),
    /* Floating point load/store */
    OPC_LWC1     = (0x31 << 26),
    OPC_LWC2     = (0x32 << 26),
    OPC_LDC1     = (0x35 << 26),
    OPC_LDC2     = (0x36 << 26),
    OPC_SWC1     = (0x39 << 26),
    OPC_SWC2     = (0x3A << 26),
    OPC_SDC1     = (0x3D << 26),
    OPC_SDC2     = (0x3E << 26),
    /* MDMX ASE specific */
    OPC_MDMX     = (0x1E << 26),
    /* Cache and prefetch */
    OPC_CACHE    = (0x2F << 26),
    OPC_PREF     = (0x33 << 26),
    /* Reserved major opcode */
    OPC_MAJOR3B_RESERVED = (0x3B << 26),
};

#if 1
static void mypc_callback_for_net( target_ulong pc, uint32_t opcode)
{
#if 0
#define OP_MTC0 0x40800000
#define OP_DMTC0 0x40a00000
int rt, rd, sel;
MIPSCPU *cpu = MIPS_CPU(current_cpu);
CPUMIPSState *env = &cpu->env;
if(((opcode & 0xffe00000) == OP_MTC0) || ((opcode & 0xffe00000) == OP_DMTC0))
{

				rt = (opcode >> 16) & 0x1f;
				rd = (opcode >> 11) & 0x1f;
				sel = opcode & 7;
					printf("pc = %llx sel %d rd %d rt %d 0x%llx\n",(long long) pc, sel, rd, rt, (long long)env->active_tc.gpr[rt]);
				if(sel == 0 && rd == 12 && env->active_tc.gpr[rt] &1 )
				{
					//exit(0);
				}
}

	CPUMIPSState *env = cpu_single_env;
	int rs, imm;
	if (pc>=mynet.pc_low && pc<mynet.pc_high)
	{
		mynet.runins++;
		pcbuf[pcbuf_pos].pc = pc;

		switch(MASK_OP_MAJOR(opcode))
		{
			/* MIPS64 opcodes */
			case OPC_LWU:
			case OPC_LD:
			case OPC_SD:
			case OPC_LW:
			case OPC_LH:
			case OPC_LHU:
			case OPC_LB:
			case OPC_LBU:
			case OPC_SW:
			case OPC_SH:
			case OPC_SB:
				rs = (opcode >> 21) & 0x1f;
				//rt = (opcode >> 16) & 0x1f;
				imm = (int16_t)opcode;
				pcbuf[pcbuf_pos].addr = env->active_tc.gpr[rs]+imm; 
				break;
			default:
				pcbuf[pcbuf_pos].addr = 0;
				break;
		}

		pcbuf_pos = (pcbuf_pos+1)&(pcbuf_size-1);
	}
#endif
}
#endif


static MemoryRegion *ddrcfg_iomem;

static void mips_qemu_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case 0x1fd00800 ... 0x1fd0082c:
		*(int *)((char *)&mynet + (addr-0x1fd00800)) = val;
		break;
		case 0x1fd00830:
		mynet.count = val;

		{
			int fd, ret;
			int len1, len2;
			static int i = 0;
			static char *filename[10];

			if(filename[i]) unlink(filename[i]);
			ret=asprintf(&filename[i], "/tmp/pc.sample%d.%d", i%10,(int)val);
			if(ret==-1) printf("error\n");
			printf("create file %s\n", filename[i]);
			fd = open(filename[i],O_CREAT|O_TRUNC|O_WRONLY, 0666);
			ret = write (fd, &mynet.skb,24);
			if(pcbuf_size-pcbuf_pos>val)
			{
				len1 = val*sizeof(mypc);
				len2 = 0;
			}
			else
			{
				len1 =  (pcbuf_size-pcbuf_pos)*sizeof(mypc);
				len2 = val*sizeof(mypc)-len1;
			}
			ret = write(fd, pcbuf + ((pcbuf_size + pcbuf_pos - val)%pcbuf_size), len1);
			if(len2)
				ret += write(fd, pcbuf, len2); 	

			if(ret!=val*sizeof(mypc)) printf("error\n");
			close(fd);
			i++;
		}

		break;
	}
}

static uint64_t mips_qemu_readl (void *opaque, hwaddr addr, unsigned size)
{
	uint32_t isr;
	uint32_t en;

	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case 0x1fe11040:
			address_space_read(&address_space_memory, 0x1fe11420, MEMTXATTRS_UNSPECIFIED, (uint8_t *)&isr, 4);
			address_space_read(&address_space_memory, 0x1fe11424, MEMTXATTRS_UNSPECIFIED, (uint8_t *)&en, 4);
			return isr & en;
			break;
		case 0x1fe11140:
		case 0x1fe11144:
		case 0x1fe11148:
		case 0x1fe1114c:
		case 0x1fe1104c:
		case 0x1fe11044:
			return 0;
		case 0x1fe11048:
			address_space_read(&address_space_memory, 0x1fe11460, MEMTXATTRS_UNSPECIFIED, (uint8_t *)&isr, 4);
			address_space_read(&address_space_memory, 0x1fe11464, MEMTXATTRS_UNSPECIFIED, (uint8_t *)&en, 4);
			return isr & en;
			break;
		case 0x1fd00800 ... 0x1fd0082c:
		return *(int *)((char *)&mynet + (addr-0x1fd00800));
		case 0x1fd00830:
		return 0;
		break;
		case 0x0ff00960:
		return 0x100;
		case 0x0ff00010:
		return 1;
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


#define BOOTPARAM_PHYADDR ((240 << 20))
#define BOOTPARAM_ADDR (0x80000000+BOOTPARAM_PHYADDR)
// should set argc,argv
//env->gpr[REG][env->current_tc]
static int set_bootparam(ram_addr_t initrd_offset,long initrd_size)
{
	char memenv[32];
	char highmemenv[32];
	const char *pmonenv[]={"cpuclock=200000000",memenv,highmemenv};
	int i;
	long params_size;
	char *params_buf;
	unsigned int *parg_env;
	int ret;

	/* Store command line.  */
	params_size = 264;
	params_buf = g_malloc(params_size);

	parg_env=(void *)params_buf;

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

	//env
	sprintf(memenv,"memsize=%d",loaderparams.ram_size>=0xf000000?240:(loaderparams.ram_size>>20));
	sprintf(highmemenv,"highmemsize=%d",loaderparams.ram_size>0x10000000?(loaderparams.ram_size>>20)-256:0);


	for(i=0;i<sizeof(pmonenv)/sizeof(char *);i++)
	{
		*parg_env++=BOOTPARAM_ADDR+ret;
		ret +=1+snprintf(params_buf+ret,256-ret,"%s",pmonenv[i]);
	}

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
	loaderparams.a0 = 2;
	loaderparams.a1 = (target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR;
	loaderparams.a2 = (target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR +12;
	
return 0;
}

static int set_bootparam1(ram_addr_t initrd_offset,long initrd_size)
{
	char memenv[32];
	char highmemenv[32];
	long params_size;
	void *params_buf;
	unsigned int *parg_env;
	int ret;

	/* Store command line.  */
	params_size = 0x100000;
	params_buf = g_malloc(params_size);

	parg_env=(void *)params_buf;

	/*
	 * pram buf like this:
	 *argv[0] argv[1] 0 env[0] env[1] ...env[i] ,0, argv[0]'s data , argv[1]'s data ,env[0]'data,...,env[i]'s dat,0
	 */

	//jump over argv and env area
	ret =(3+1)*4;
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

	//env

	sprintf(memenv,"%d",loaderparams.ram_size>0xf000000?240:(loaderparams.ram_size>>20));
	sprintf(highmemenv,"%d",loaderparams.ram_size>0x10000000?(loaderparams.ram_size>>20)-256:0);
	setenv("memsize", memenv, 1);
	setenv("highmemsize", highmemenv, 1);

	ret = ((ret+32)&~31);

	boot_params_buf = (void *)(params_buf+ret);
	boot_params_p = boot_params_buf + align(sizeof(struct boot_params));

	init_boot_param(boot_params_buf);
	printf("param len=%ld\n", boot_params_p-params_buf);

	rom_add_blob_fixed("params", params_buf, params_size,
			BOOTPARAM_PHYADDR);
	loaderparams.a0 = 2;
	loaderparams.a1 = (target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR;
	loaderparams.a2 = (target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR + ret;
        printf("env %x\n", BOOTPARAM_PHYADDR + ret);
return 0;
}


static int64_t load_kernel(void)
{
    int64_t entry, kernel_low, kernel_high;
    long kernel_size, initrd_size;
    ram_addr_t initrd_offset;

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
	    if(getenv("INITRD_OFFSET")) initrd_offset=strtoul(getenv("INITRD_OFFSET"),0,0);
            initrd_size = load_image_targphys(loaderparams.initrd_filename,
                                     initrd_offset,ram_size-initrd_offset); //qemu_get_ram_ptr
        }
        if (initrd_size == (target_ulong) -1) {
            fprintf(stderr, "qemu: could not load initial ram disk '%s'\n",
                    loaderparams.initrd_filename);
            exit(1);
        }
    }


	if(getenv("OLDENV"))
	 set_bootparam(initrd_offset, initrd_size);
	else
	 set_bootparam1(initrd_offset, initrd_size);
	
return entry;
}
static void main_cpu_reset(void *opaque)
{
	ResetData *s = (ResetData *)opaque;
	CPUMIPSState *env = &s->cpu->env;

	cpu_reset(CPU(s->cpu));
	//env->CP0_IntCtl = 0xfc000000;
	env->active_tc.PC = s->vector;
	env->active_tc.gpr[4]=loaderparams.a0;
	env->active_tc.gpr[5]=loaderparams.a1;
	env->active_tc.gpr[6]=loaderparams.a2;
}


static void *ls2k_intctl_init(MemoryRegion *mr, hwaddr addr, qemu_irq parent_irq);

static const int sector_len = 32 * 1024;

static PCIBus *pcibus_ls2k_init(int busno,qemu_irq *pic, int (*board_map_irq)(PCIDevice *d, int irq_num));



static int ddr2config = 0;

static CPUUnassignedAccess real_do_unassigned_access;
static void mips_ls2k_do_unassigned_access(CPUState *cpu, hwaddr addr,
                                           bool is_write, bool is_exec,
                                           int opaque, unsigned size)
{
    if (!is_exec) {
        /* ignore invalid access (ie do not raise exception) */
        return;
    }
    (*real_do_unassigned_access)(cpu, addr, is_write, is_exec, opaque, size);
}

static void mips_ls2k_init(MachineState *machine)
{
	ram_addr_t ram_size = machine->ram_size;
	const char *cpu_model = machine->cpu_model;
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
	PCIBus *pci_bus[4];
	DriveInfo *flash_dinfo=NULL;
	CPUClass *cc;
	MemoryRegion *iomem_root = g_new(MemoryRegion, 1);
	AddressSpace *as = g_new(AddressSpace, 1);


	/* init CPUs */
	if (cpu_model == NULL) {
#ifdef TARGET_MIPS64
		cpu_model = "loongson2k";
#else
		cpu_model = "loongson2k";
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
		cc->do_unassigned_access = mips_ls2k_do_unassigned_access;

		reset_info = g_malloc0(sizeof(ResetData));
		reset_info->cpu = cpu;
		reset_info->vector = env->active_tc.PC;
		qemu_register_reset(main_cpu_reset, reset_info);

		/* allocate RAM */
	memory_region_init_ram(ram, NULL, "mips_r4k.ram", ram_size, &error_fatal);
	vmstate_register_ram_global(ram);

	MemoryRegion *ram1 = g_new(MemoryRegion, 1);
	memory_region_init_alias(ram1, NULL, "lowmem", ram, 0, 0x10000000);
	memory_region_add_subregion(address_space_mem, 0, ram1);
	memory_region_add_subregion(address_space_mem, 0x100000000ULL, ram);


        //memory_region_init_iommu(iomem_root, NULL, &ls1a_pcidma_iommu_ops, "ls2k axi", UINT32_MAX);
        memory_region_init(iomem_root, NULL,  "ls2k axi", UINT32_MAX);
        memory_region_init(iomem_root, NULL,  "ls2k axi", UINT32_MAX);
	address_space_init(as,iomem_root, "ls2k axi memory");

	MemoryRegion *ram2 = g_new(MemoryRegion, 1);
	memory_region_init_alias(ram2, NULL, "lowmem", ram, 0, ram_size);
        memory_region_add_subregion(iomem_root, 0, ram2);


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
	cpu_mips_irq_init_cpu(cpu);
	cpu_mips_clock_init(cpu);


	/* Register 64 KB of IO space at 0x1f000000 */
	//isa_mmio_init(0x1ff00000, 0x00010000);
	//isa_mem_base = 0x10000000;
	ls2k_irq =ls2k_intctl_init(get_system_memory(), 0x1Fe11420, env->irq[2]);
	ls2k_irq1=ls2k_intctl_init(get_system_memory(), 0x1Fe11460, env->irq[3]);


	if (serial_hds[0])
		serial_mm_init(address_space_mem, 0x1fe00000, 0,ls2k_irq[0],115200,serial_hds[0], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[1])
		serial_mm_init(address_space_mem, 0x1fe00100, 0,ls2k_irq[0],115200,serial_hds[1], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[2])
		serial_mm_init(address_space_mem, 0x1fe00200, 0,ls2k_irq[0],115200,serial_hds[2], DEVICE_NATIVE_ENDIAN);

	if (serial_hds[3])
		serial_mm_init(address_space_mem, 0x1fe00300, 0,ls2k_irq[0],115200,serial_hds[3], DEVICE_NATIVE_ENDIAN);

	sysbus_create_simple("ls2h_fb", 0x1fe50000, NULL);



	pci_bus[0]=pcibus_ls2k_init(0, &ls2k_irq1[20],pci_ls2k_map_irq);



#if 0
	sysbus_create_simple("ls2k_acpi",0x1fe7c000, ls2k_irq[0]);
#endif

	if (nb_nics) {
#if 1
	int i;
	int devfn;
	   for(i=0,devfn=PCI_DEVFN(0, 0);i<nb_nics;i++,devfn += PCI_DEVFN(1, 0))
	  {
		PCIDevice *pci_dev;
		DeviceState *dev;
            //dev = pci_nic_init(&nd_table[i],nd_table[i].model?:"e1000","01:0b");
		pci_dev = pci_create(pci_bus[0], devfn, "e1000");
		dev = DEVICE(pci_dev);
		qdev_set_nic_properties(dev, &nd_table[i]);
    		DeviceClass *dc = DEVICE_GET_CLASS(dev);
    		PCIDeviceClass *k = PCI_DEVICE_CLASS(DEVICE_CLASS(dc));
    		k->romfile = NULL;
    		dc->vmsd = NULL;
		qdev_init_nofail(dev);

	    	printf("nb_nics=%d dev=%p\n", nb_nics, pci_dev);
	  }
#endif
	}

#if 0
	{
		sysbus_create_simple("ls2k_wdt", 0x1fe7c060, NULL);
	}
#endif

	{
		DeviceState *dev,*dev1;
		void *bus;
		qemu_irq cs_line;
		dev=sysbus_create_simple("ls1a_spi",0x1fe70000, ls2k_irq[8]);
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



#if 0
	{
		DeviceState *dev;
		SysBusDevice *s;
		dev = qdev_create(NULL, "ls2k_ac97");
		printf("ac97 dev=%p\n",dev);
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x1fe74000);
		sysbus_connect_irq(s, 0, ls2k_irq[14]);
		sysbus_connect_irq(s, 1, ls2k_irq[15]);
	}
#endif

		sysbus_create_simple("ls1a_dma",0x1fe10c00, NULL);
		sysbus_create_simple("ls1a_nand",0x1fe06000, ls2k_irq[12]);


#if 1
	{
		DeviceState *dev;
		SysBusDevice *s;
		dev = qdev_create(NULL, "ls1a_rtc");
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x1fe03000);
		sysbus_connect_irq(s, 0, ls2k_irq[14]);
		sysbus_connect_irq(s, 1, ls2k_irq[15]);
		sysbus_connect_irq(s, 2, ls2k_irq[16]);
	}
#endif

	{
		DeviceState *dev;
		void *bus;
		dev=sysbus_create_simple("ls1a_i2c",0x1fe01000, ls2k_irq[7]);
		bus = qdev_get_child_bus(dev, "i2c");
		i2c_create_slave(bus, "ds1338", 0x68);
	}

	{
		DeviceState *dev;
		void *bus;
		dev=sysbus_create_simple("ls1a_i2c",0x1fe01800, ls2k_irq[8]);
		bus = qdev_get_child_bus(dev, "i2c");
		i2c_create_slave(bus, "ds1338", 0x68);
	}


	{

                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fd00210, "0x1fd00210", 0x4);
                memory_region_add_subregion(address_space_mem, 0x1fd00210, iomem);
		/*ins*/
                iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fd00800, "0x1fd00800", 0x34);
                memory_region_add_subregion(address_space_mem, 0x1fd00800, iomem);
	}


	{
                ddrcfg_iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(ddrcfg_iomem, NULL, &mips_qemu_ops, (void *)0x0ff00000, "ddr", 0x100000);
	}

	{

                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fe11040, "0x1fe11040", 16);
                memory_region_add_subregion(address_space_mem, 0x1fe11040, iomem);
		iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fe11140, "0x1fe11140", 16);
                memory_region_add_subregion(address_space_mem, 0x1fe11140, iomem);
	}


#if 0
	{
		DeviceState *dev;
		SysBusDevice *s;
		BusState *hdabus;
		DeviceState *codec;
		dev = qdev_create(NULL, "intel-hda");
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x1fe20000);
		sysbus_connect_irq(s, 0, ls2k_irq1[25]);
		hdabus = QLIST_FIRST(&dev->child_bus);
		codec = qdev_create(hdabus, "hda-duplex");
		qdev_init_nofail(codec);
	}
#endif

	pcbuf_size = 0x100000;
	pcbuf_pos = 0;
	pcbuf = malloc(pcbuf_size*sizeof(mypc));
	mypc_callback =  mypc_callback_for_net;
}


static void mips_machine_init(MachineClass *mc)
{
    mc->desc = "mips ls2k platform";
    mc->init = mips_ls2k_init;
}

DEFINE_MACHINE("ls2k", mips_machine_init)

//------------
#include "hw/pci/pci.h"
#include "hw/pci/pci_host.h"
#include "hw/pci/pci_bus.h"


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

#include "ls2k_int.c"
//-------------------------
// pci bridge
//-----------------

static int pci_ls2k_map_irq(PCIDevice *d, int pin)
{
	int dev=(d->devfn>>3)&0x1f;
	int fn=d->devfn& 7;

	  switch(dev)
	  {
		case 2:
		/*APB 2*/
		break;

		case 3:
		/*GMAC0 3 0*/
		/*GMAC1 3 1*/
		return (fn==0)?12:14;

		case 4:
		/*
		OTG: 4 0
		EHCI: 4 1
		OHCI: 4 2
		*/
		 return (fn == 0)? 49:(fn == 1)? 50:51;
		break;

		case 5:
		/*GPU*/
		 return 29;
		break;

		case 6:
		/*DC*/
		 return 28;
		break;

		case 7:
		/*HDA*/
		 return 4;
		break;

		case 8:
		/*SATA*/
		 return 19;
		break;

		case 9:
		/*PCIE PORT 0*/
		 return 32+pin-1;;
		break;

		case 10:
		/*PCIE PORT 1*/
		 return 36+pin-1;
		break;

		case 11:
		/*PCIE PORT 2*/
		break;

		case 12:
		/*PCIE PORT 3*/
		break;

		case 13:
		/*PCIE1 PORT 0*/
		break;

		case 14:
		/*PCIE1 PORT 1*/
		break;

		case 15:
		/*DMA*/
		break;

	  }
	
	return  0;
}

static void pci_ls2k_set_irq(void *opaque, int irq_num, int level)
{
	if(irq_num<32)
		qemu_set_irq(ls2k_irq[irq_num],level);
	else 
		qemu_set_irq(ls2k_irq1[irq_num-32],level);
}
/*self pci header*/
#define LS2K_PCIE_PORT_HEAD_BASE_PORT(portnum)  (0x18114000 + (portnum << 22))
/*devices pci header*/
#define LS3H_PCIE_DEV_HEAD_BASE_PORT(portnum)   (0x18116000 + (portnum << 22))
/*pci map*/
#define LS2K_PCIE_REG_BASE_PORT(portnum)        (0x18118000 + (portnum << 22))
#define LS2K_PCIE_PORT_REG_STAT1		0xC
#define LS2K_PCIE_PORT_REG_CTR0			0x0
#define LS2K_PCIE_PORT_REG_CFGADDR		0x24
#define LS2K_PCIE_PORT_REG_CTR_STAT		0x28


#define TYPE_BONITO_PCI_HOST_BRIDGE "ls2k-pcihost"
typedef struct BonitoState BonitoState;

#define BONITO_PCI_HOST_BRIDGE(obj) \
    OBJECT_CHECK(BonitoState, (obj), TYPE_BONITO_PCI_HOST_BRIDGE)

typedef struct PCIBonitoState
{
    /*< private >*/
    	PCIBridge parent_obj;
    /*< public >*/
	BonitoState *pcihost;
	MemoryRegion iomem;
	MemoryRegion conf_mem;
	MemoryRegion data_mem;
	struct pcilocalreg{
		/*0*/
		unsigned int portctr0;
		unsigned int portctr1;
		unsigned int portstat0;
		unsigned int portstat1;
		/*0x10*/
		unsigned int usrmsgid;
		unsigned int nouse;
		unsigned int portintsts;
		unsigned int portintclr;
		/*0x20*/
		unsigned int portintmsk;
		unsigned int portcfg;
		unsigned int portctrsts;
		unsigned int physts;
		/*0x30*/
		unsigned int nouse1[2];
		unsigned int usrmsg0;
		unsigned int usrmsg1;
		/*0x40*/
		unsigned int usrmsgsend0;
		unsigned int usrmsgsend1;
		unsigned int noused2[5];
		/*0x5c*/
		unsigned int msi;
		unsigned int noused3[2];
		/*0x68*/
		unsigned int addrmsk;
		unsigned int addrmsk1;
		/*0x70*/
		unsigned int addrtrans;
		unsigned int addrtrans1;
		unsigned int dataload0;
		unsigned int dataload1;
	} mypcilocalreg;
} PCIBonitoState;

struct BonitoState {
    SysBusDevice busdev;
    PCIBus *bus;
    qemu_irq *pic;
    PCIBonitoState *pci_dev;
    MemoryRegion iomem_mem;
    MemoryRegion iomem_submem;
    MemoryRegion iomem_io;
    AddressSpace as_mem;
    AddressSpace as_io;
   int (*pci_map_irq)(PCIDevice *d, int irq_num);
};

static void pci_ls2k_config_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	PCIBonitoState *s = opaque;
	//   PCIDevice *d = PCI_DEVICE(s);
	BonitoState *phb = s->pcihost;

	if(!(addr&0x1000000)) addr &= 0xffffff;

	pci_data_write(phb->bus,  addr, val, size);
}

static uint64_t pci_ls2k_config_readl (void *opaque, hwaddr addr, unsigned size)
{
	PCIBonitoState *s = opaque;
	//  PCIDevice *d = PCI_DEVICE(s);
	BonitoState *phb = s->pcihost;
	uint32_t val;

	if(!(addr&0x1000000)) addr &= 0xffffff;


	val = pci_data_read(phb->bus, addr, size);
	printf("pci_ls2k_config_readl 0x%x 0x%x\n", (int)addr, val);
	return val;
}


static const MemoryRegionOps pci_ls2k_config_ops = {
    .read = pci_ls2k_config_readl,
    .write = pci_ls2k_config_writel,
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
    PCIBonitoState *s = DO_UPCAST(PCIBonitoState, parent_obj.parent_obj, dev);
    SysBusDevice *sysbus = SYS_BUS_DEVICE(s->pcihost);

    pci_bridge_initfn(dev, TYPE_PCI_BUS);

    /* set the north bridge pci configure  mapping */
    memory_region_init_io(&s->conf_mem, NULL, &bonito_pciconf_ops, s,
                          "north-bridge-pci-config", 0x100);
    sysbus_init_mmio(sysbus, &s->conf_mem);

    /* set the south bridge pci configure  mapping */
    memory_region_init_io(&s->data_mem, NULL, &pci_ls2k_config_ops, s,
                          "south-bridge-pci-config", 0x2000000);
    sysbus_init_mmio(sysbus, &s->data_mem);

    pci_config_set_prog_interface(dev->config, PCI_CLASS_BRIDGE_PCI_INF_SUB);
    /* set the default value of north bridge pci config */
//hw/xio3130_downstream.c

    pci_set_word(dev->config + PCI_COMMAND,
                 PCI_COMMAND_MEMORY|PCI_COMMAND_IO);
    pci_set_word(dev->config + PCI_STATUS,
                 PCI_STATUS_FAST_BACK | PCI_STATUS_66MHZ |
                 PCI_STATUS_DEVSEL_MEDIUM);
    pci_set_word(dev->config + PCI_SUBSYSTEM_VENDOR_ID, 0x0000);
    pci_set_word(dev->config + PCI_SUBSYSTEM_ID, 0x0000);

    pci_set_byte(dev->config + PCI_INTERRUPT_LINE, 0x00);
    pci_set_byte(dev->config + PCI_INTERRUPT_PIN, 0x01);
    pci_set_byte(dev->config + PCI_MIN_GNT, 0x3c);
    pci_set_byte(dev->config + PCI_MAX_LAT, 0x00);
    pci_set_word(dev->config + PCI_CLASS_DEVICE, 0x0b30);


    return 0;
}


static void bonito_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->init = bonito_initfn;
    k->exit = pci_bridge_exitfn;
    k->vendor_id = 0xdf53;
    k->device_id = 0x00d5;
    k->revision = 0x01;
    k->class_id = PCI_CLASS_BRIDGE_HOST;
    k->is_bridge = 1;
    k->config_write = pci_bridge_write_config;
    dc->reset = pci_bridge_reset;
    dc->desc = "Host bridge";
//pci_bridge_dev_initfn
}

static const TypeInfo bonito_info = {
    .name          = "LS2K_Bonito",
    .parent        = TYPE_PCI_BRIDGE,
    .instance_size = sizeof(PCIBonitoState),
    .class_init    = bonito_class_init,
};

static AddressSpace *pci_dma_context_fn(PCIBus *bus, void *opaque, int devfn);

static PCIBus *pcibus_ls2k_init(int busno, qemu_irq *pic, int (*board_map_irq)(PCIDevice *d, int irq_num))
{
    DeviceState *dev;
    BonitoState *pcihost;
    PCIBonitoState *s;
    PCIDevice *d;
    SysBusDevice *sysbus;
    PCIBridge *br;
    PCIBus *bus2;

    dev = qdev_create(NULL, TYPE_BONITO_PCI_HOST_BRIDGE);
    pcihost = BONITO_PCI_HOST_BRIDGE(dev);
    pcihost->pic = pic;
    pcihost->pci_map_irq = board_map_irq;
    qdev_init_nofail(dev);

    /* set the pcihost pointer before bonito_initfn is called */
    //d = pci_create(phb->bus, PCI_DEVFN(0, 0), "LS2K_Bonito");
    d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(9, 0), true, "LS2K_Bonito");

    s = DO_UPCAST(PCIBonitoState, parent_obj.parent_obj, d);
    s->pcihost = pcihost;
    pcihost->pci_dev = s;
    br = PCI_BRIDGE(d);
    pci_bridge_map_irq(br, "Advanced PCI Bus secondary bridge 1", board_map_irq);
    qdev_init_nofail(DEVICE(d));
    bus2 = pci_bridge_get_sec_bus(br);

    pci_setup_iommu(bus2, pci_dma_context_fn, pcihost);

    d = pci_create(pcihost->bus, PCI_DEVFN(3, 0), "pci-synopgmac");
    qdev_init_nofail(DEVICE(d));
    pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
    pci_set_word(d->config + PCI_DEVICE_ID, 0x7a03);


    d = pci_create(pcihost->bus, PCI_DEVFN(4, 0), "pci-ohci");
    qdev_init_nofail(DEVICE(d));
    pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
    pci_set_word(d->config + PCI_DEVICE_ID, 0x7a04);

#if 0
    d = pci_create(pcihost->bus, PCI_DEVFN(4, 1), "pci-ehci-usb");
    qdev_init_nofail(DEVICE(d));
    pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
    pci_set_word(d->config + PCI_DEVICE_ID, 0x7a14);

    d = pci_create(pcihost->bus, PCI_DEVFN(4, 2), "pci-ohci");
    qdev_init_nofail(DEVICE(d));
    pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
    pci_set_word(d->config + PCI_DEVICE_ID, 0x7a24);
#endif

    sysbus = SYS_BUS_DEVICE(s->pcihost);
     /*self header*/
    sysbus_mmio_map(sysbus, 0, 0xfe00004800);
     /*devices header*/
    sysbus_mmio_map(sysbus, 1, 0x1a000000);

    memory_region_add_subregion(get_system_memory(), 0x10000000UL, &pcihost->iomem_submem);
    memory_region_add_subregion(get_system_memory(), 0x18000000UL, &pcihost->iomem_io);

//pci-synopgmac


    return bus2;
}


static AddressSpace *pci_dma_context_fn(PCIBus *bus, void *opaque, int devfn)
{
    BonitoState *pcihost = opaque;
    return &pcihost->as_mem;
}


/* Handle PCI-to-system address translation.  */
/* TODO: A translation failure here ought to set PCI error codes on the
   Pchip and generate a machine check interrupt.  */
static IOMMUTLBEntry ls2k_pciedma_translate_iommu(MemoryRegion *iommu, hwaddr addr, bool is_write)
{
    //BonitoState *pcihost = container_of(iommu, BonitoState, iomem_mem);
    IOMMUTLBEntry ret;

    ret = (IOMMUTLBEntry) {
        .target_as = &address_space_memory,
        .translated_addr = addr,
        .addr_mask = -1ULL,
        .perm = IOMMU_RW,
    };


    return ret;
}

static const MemoryRegionIOMMUOps ls2k_pciedma_iommu_ops = {
    .translate = ls2k_pciedma_translate_iommu,
};

static int bonito_pcihost_initfn(SysBusDevice *dev)
{
    BonitoState *pcihost;
    PCIHostState *phb;
    pcihost = BONITO_PCI_HOST_BRIDGE(dev);

    memory_region_init(&pcihost->iomem_mem, OBJECT(pcihost), "system", INT32_MAX);
    address_space_init(&pcihost->as_mem, &pcihost->iomem_mem, "pcie memory");

    /* Host memory as seen from the PCI side, via the IOMMU.  */
    memory_region_init_iommu(&pcihost->iomem_mem, OBJECT(dev), &ls2k_pciedma_iommu_ops, "iommu-ls2kpcie", UINT64_MAX);

    memory_region_init_alias(&pcihost->iomem_submem, NULL, "pcisubmem", &pcihost->iomem_mem, 0x10000000, 0x2000000);

    memory_region_init(&pcihost->iomem_io, OBJECT(pcihost), "system", 0x10000);
    address_space_init(&pcihost->as_io, &pcihost->iomem_io, "pcie io");

    phb = PCI_HOST_BRIDGE(dev);
    pcihost->bus = phb->bus = pci_register_bus(DEVICE(dev), "pci",
                                pci_ls2k_set_irq, pcihost->pci_map_irq, pcihost->pic,
                                &pcihost->iomem_mem, &pcihost->iomem_io,
                                PCI_DEVFN(0, 0), 4, TYPE_PCI_BUS);


    pci_setup_iommu(pcihost->bus, pci_dma_context_fn, pcihost);

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
#define LOONGSON_2H
#include "loongson_bootparam.c"
