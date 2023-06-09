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
#include "hw/ide/pci.h"
#include "hw/ide/ahci_internal.h"
#include "hw/pci/pcie_host.h"
#include "hw/pci/pcie_port.h"
#include "loongson_bootparam.h"
#include <stdlib.h>
#include "loongson1a500_rom.h"
#include "hw/timer/hpet.h"
#include "sysemu/device_tree.h"
#include "libfdt.h"
#include "monitor/qdev.h"
extern target_ulong mypc;

#define PHYS_TO_VIRT(x) ((x)>=0x20000000?((x) | 0x9800000000000000ULL):((x) | ~(target_ulong)0x7fffffff))

#define VIRT_TO_PHYS_ADDEND (-((int64_t)(int32_t)0x80000000))

#define MAX_IDE_BUS 2
#define TARGET_REALPAGE_MASK (TARGET_PAGE_MASK<<2)

#define ALIAS_REGION_FROM_RAS_TO_RA(REGIONA,ADDR,SIZE,REGIONB,ALIAS) \
({\
    MemoryRegion *alias_mr = g_new(MemoryRegion, 1); \
    memory_region_init_alias(alias_mr, NULL, NULL, REGIONA, ADDR, SIZE); \
    memory_region_add_subregion(REGIONB, ALIAS, alias_mr); \
alias_mr;})

#define _str(x) #x
#define str(x) _str(x)
#define SIMPLE_OPS(ADDR,SIZE) \
	({\
                MemoryRegion *iomem = g_new(MemoryRegion, 1);\
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)ADDR, str(ADDR) , SIZE);\
                memory_region_add_subregion_overlap(address_space_mem, ADDR, iomem, 1);\
		iomem;\
	})

static const int ide_iobase[2] = { 0x1f0, 0x170 };
static const int ide_iobase2[2] = { 0x3f6, 0x376 };
static const int ide_irq[2] = { 14, 15 };
static qemu_irq *ls1a500_irq,*ls1a500_irq1;

/* i8254 PIT is attached to the IRQ0 at PIC i8259 */

static struct _loaderparams {
    uint64_t ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
    target_ulong a0,a1,a2;
} loaderparams;

static void *boot_params_buf;
static void *boot_params_p;
static PCIBus *ls1a500_pci_bus;
#define align(x) (((x)+15)&~15)
static int pci_ls1a500_map_irq(PCIDevice *d, int irq_num);


extern void (*mypc_callback)(target_ulong pc, uint32_t opcode);

void *ls1gpa_mmci_init(MemoryRegion *sysmem,
                hwaddr base,
                BlockBackend *blk, qemu_irq irq);


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


void ls1gpa_sdio_set_dmaaddr(uint32_t val);
void __attribute__((weak)) ls1gpa_sdio_set_dmaaddr(uint32_t val)
{
}

#define GPUBASE 0xd0000000
static MemoryRegion *ddrcfg_iomem;
static int reg424 = 0x800;

#ifdef DEBUG_PCIEDMA
static AddressSpace *ls1a500_pci_as;
static unsigned int ls1a500_pci_addr;
#endif

static uint64_t gpio_ov;
static SysBusDevice *liodev[2];
static MemoryRegion *liomr;
static uint32_t ls2k_cpu_irqsts(int cpu, int i);
#define GPIO0_ADDR 0x1fe10430
#define GPIO0_OUT_ADDR 0x1fe10440

static void mips_qemu_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case GPIO0_OUT_ADDR ... GPIO0_OUT_ADDR+7:
		memcpy((void *)&gpio_ov + addr - GPIO0_OUT_ADDR, (void *)&val, size);
		{
			int i = (gpio_ov >> 48) & 1;
			if (liomr->alias != liodev[i]->mmio[0].memory || liomr->alias_offset !=(((gpio_ov>>44)&7)*0x2000000) ) {
				memory_region_del_subregion(get_system_memory(), liomr);
    g_free(liomr);

    liomr = ALIAS_REGION_FROM_RAS_TO_RA(liodev[i]->mmio[0].memory, ((gpio_ov>>44)&7)*0x2000000, 0x2000000, get_system_memory(), 0x1c000000);
			}
		}
		break;
		case 0x1fe10c30:
		ls1gpa_sdio_set_dmaaddr(val);
		break;
		case 0x1fe10100:
			reg424 = val;
			memory_region_transaction_begin();
			if(ddrcfg_iomem->container == get_system_memory())
				memory_region_del_subregion(get_system_memory(), ddrcfg_iomem);

			if((val&0x800) == 0)
			{
				memory_region_add_subregion_overlap(get_system_memory(), 0x0ff00000, ddrcfg_iomem, 1);
			}

			memory_region_transaction_commit();
			break;
#ifdef DEBUG_PCIEDMA
		case 0x1fef0000:
		dma_memory_write(ls1a500_pci_as, ls1a500_pci_addr, &val, 4);
		break;
		case 0x1fef0004:
		ls1a500_pci_addr = val;
		break;
#endif
	}
}

static uint64_t mips_qemu_readl (void *opaque, hwaddr addr, unsigned size)
{
	uint64_t val = 0;

	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case 0x1fe11040:
			val = ls2k_cpu_irqsts(0, 0);
			break;
		case 0x1fe11048:
			val = ls2k_cpu_irqsts(0, 1);
			break;
		case 0x1f06c120:
			val = 0x55330000;
			break;
		case GPIO0_OUT_ADDR ... GPIO0_OUT_ADDR+7:
		memcpy((void *)&val , (void *)&gpio_ov + addr - GPIO0_OUT_ADDR, size);
		break;
#ifdef DEBUG_PCIEDMA
		case 0x1fef0000:
		dma_memory_read(ls1a500_pci_as, ls1a500_pci_addr, &val, 4);
		return val;
		break;
		case 0x1fef0004:
		return ls1a500_pci_addr;
		break;
#endif
		case GPUBASE+4:
		case GPUBASE+0:
		case GPUBASE+0x100:
		return random();
		case 0x0ff00960:
		return 0x100;
		case 0x0ff00010:
		return 1;
		case 0x1fe104b0:
		return 0x10000;
		case 0x1fe104c0:
		return 0x10000;
		case 0x1fe10480:
		return 0x50010c85;
		case 0x1fe10484:
		return 0x00000450;
		case 0x1fe10488:
		return 0x00000002;
		case 0x1fe1048c:
		return 0;
		case 0x1fe10490:
		return 0x10010c87;
		case 0x1fe10494:
		return 0x00000440;
		case 0x1fe10498:
		return 0x01c00004;
		case 0x1fe1049c:
		return 00064000;
		case 0x1fe104a0:
		return 0x10000;
		case 0x0ff00160:
		return 0x0f000000;
		case 0x0ff00184:
		return 0xffff00;
		case 0x0ff00188:
		return 0x1000000;
		case 0x0ff0018c:
		return 0x1;
		case 0x1fe10100:
		return reg424;
		default:
		return	random();
		break;
	}
	return val;
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
#define PBUF_SIZE 1024
	params_size = PBUF_SIZE + 8;
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
	ret +=1+snprintf(params_buf+ret,PBUF_SIZE-ret,"g");
	//argv1
	*parg_env++=BOOTPARAM_ADDR+ret;
	if (initrd_size > 0) {
		ret +=1+snprintf(params_buf+ret,PBUF_SIZE-ret, "rd_start=0x%llx rd_size=%li %s",
				PHYS_TO_VIRT(initrd_offset),
				initrd_size, loaderparams.kernel_cmdline);
	} else {
		ret +=1+snprintf(params_buf+ret, PBUF_SIZE-ret, "%s", loaderparams.kernel_cmdline);
	}
	//argv2
	*parg_env++=0;

	//env
	sprintf(memenv,"memsize=%d",(int)(loaderparams.ram_size>=0xf000000?240:(int)(loaderparams.ram_size>>20)));
	sprintf(highmemenv,"highmemsize=%d",(int)(loaderparams.ram_size>0x10000000?(int)(loaderparams.ram_size>>20)-256:0));


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
	loaderparams.a0 = 2;
	loaderparams.a1 = (target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR;
	loaderparams.a2 = (target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR +12;
	
return 0;
}

static int set_bootparam1(ram_addr_t initrd_offset,long initrd_size, char *dtb)
{
	char memenv[32];
	char highmemenv[32];
	long params_size;
	void *params_buf;
	unsigned int *parg_env;
	int ret;
	char *cmdline;

	/* Store command line.  */
	//params_size = 0x100000;
#undef PBUF_SIZE
#define PBUF_SIZE 0x100000
	params_size = PBUF_SIZE;
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
	ret +=1+snprintf(params_buf+ret,PBUF_SIZE-ret,"g");
	//argv1
	*parg_env++=BOOTPARAM_ADDR+ret;
	cmdline = params_buf+ret;
	if (initrd_size > 0) {
		ret +=1+snprintf(params_buf+ret,PBUF_SIZE-ret, "rd_start=0x%llx rd_size=%li %s",
				PHYS_TO_VIRT(initrd_offset),
				initrd_size, loaderparams.kernel_cmdline);
	} else {
		ret +=1+snprintf(params_buf+ret, PBUF_SIZE-ret, "%s", loaderparams.kernel_cmdline);
	}
	//argv2
	*parg_env++=0;

	//env

	sprintf(memenv,"%d",(int)(loaderparams.ram_size>0xf000000?240:(loaderparams.ram_size>>20)));
	sprintf(highmemenv,"%d",(int)(loaderparams.ram_size>0x10000000?(loaderparams.ram_size>>20)-256:0));
	setenv("memsize", memenv, 1);
	setenv("highmemsize", highmemenv, 1);

	ret = ((ret+32)&~31);

	boot_params_buf = (void *)(params_buf+ret);
	boot_params_p = boot_params_buf + align(sizeof(struct boot_params));

	init_boot_param(boot_params_buf);
	printf("param len=%ld\n", boot_params_p-params_buf);

	loaderparams.a0 = 2;
	loaderparams.a1 = (target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR;
	//loaderparams.a2 = (target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR + ret;
	loaderparams.a2 = 0;

	if(dtb)
	{
	int size;
	void *fdt;
	int err;
        uint64_t ram_low_sz, ram_high_sz;
	
	ret = boot_params_p-params_buf;
	loaderparams.a2 = (target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR + ret;
	fdt = load_device_tree(dtb, &size);
        printf("fdt %x %p\n", BOOTPARAM_PHYADDR + ret, fdt);

	err = qemu_fdt_setprop_string(fdt, "/chosen", "bootargs", cmdline);
	if (err < 0) {
		fprintf(stderr, "couldn't set /chosen/bootargs\n");
		exit(-1);
	}


	ram_low_sz = loaderparams.ram_size>=0x0f000000?0x0ee00000:loaderparams.ram_size;
	if(fdt_path_offset (fdt, "/soc/gpu@0x1f000000") >= 0){
	ram_high_sz = loaderparams.ram_size>0x30000000?loaderparams.ram_size-0x30000000:0;
	}
	else
	{
	ram_high_sz = loaderparams.ram_size>0x10000000?loaderparams.ram_size-0x10000000:0;
	}
	
	qemu_fdt_setprop_sized_cells(fdt, "/memory", "reg",
			2, 0x00200000, 2, ram_low_sz,
			2, 0x90000000, 2,  ram_high_sz);
        ret = boot_params_p - params_buf;
        params_size = ret + size + 8;
        params_buf = g_realloc(params_buf, params_size);
        memcpy(params_buf + ret, fdt, size);

	qemu_fdt_dumpdtb(fdt, size);
	}

	rom_add_blob_fixed("params", params_buf, params_size,
			BOOTPARAM_PHYADDR);
return 0;
}


static int64_t load_kernel(char *dtb)
{
    int64_t entry, kernel_low, kernel_high;
    long kernel_size, initrd_size;
    ram_addr_t initrd_offset;

	if(getenv("BOOTROM"))
	{
            kernel_size = load_image_targphys(loaderparams.kernel_filename,
                                     (kernel_high = strtoul(getenv("BOOTROM"),0,0)),ram_size); //qemu_get_ram_ptr
	   kernel_high +=   kernel_size;
	   entry = 0;
	}
	else
	{
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
	 set_bootparam1(initrd_offset, initrd_size, dtb);
	
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

static void *ls2k_intctl_init(MemoryRegion *mr, hwaddr addr, qemu_irq *parent_irq, MemoryRegion *pcimr);

static const int sector_len = 32 * 1024;

static PCIBus **pcibus_ls1a500_init(int busno,qemu_irq *pic, int (*board_map_irq)(PCIDevice *d, int irq_num), MemoryRegion *ram);



static int ddr2config = 0;

static CPUUnassignedAccess real_do_unassigned_access;
static void mips_ls1a500_do_unassigned_access(CPUState *cpu, hwaddr addr,
                                           bool is_write, bool is_exec,
                                           int opaque, unsigned size)
{
    if (!is_exec) {
        /* ignore invalid access (ie do not raise exception) */
        return;
    }
    (*real_do_unassigned_access)(cpu, addr, is_write, is_exec, opaque, size);
}

static CPUMIPSState *mycpu;
static int ls1a500_cpu_irq;
static int ls1a500_cpu_irq1;
static int ls1a500_cpu_irq_old;

static void ls1a500_set_cpuirq(void *opaque, int irq, int level)
{
	int *p = opaque?&ls1a500_cpu_irq1:&ls1a500_cpu_irq;
	int ls1a500_cpu_irq_new;
	if (irq >= 4)
		return;

	if (level) {
		*p |= 1<<irq;
	} else {
		*p &= ~(1<<irq);
	}

	ls1a500_cpu_irq_new =  ls1a500_cpu_irq|ls1a500_cpu_irq1;

	if( ls1a500_cpu_irq_new != ls1a500_cpu_irq_old)
	{
		if(ls1a500_cpu_irq_new&(1<<irq))
		{
			qemu_irq_raise(mycpu->irq[2+(irq%4)]);
		}
		else
		{
			qemu_irq_lower(mycpu->irq[2+(irq%4)]);
		}

		ls1a500_cpu_irq_old = ls1a500_cpu_irq|ls1a500_cpu_irq1;
	}
}


static void mips_ls1a500_init(MachineState *machine)
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
	ResetData *reset_info[2];
	PCIBus **pci_bus;
	DriveInfo *flash_dinfo=NULL;
	CPUClass *cc;
	MemoryRegion *iomem_root = g_new(MemoryRegion, 1);
	AddressSpace *as = g_new(AddressSpace, 1);
	int i;
	qemu_irq *cpu_irq;
	qemu_irq *cpu_irq1;
	cpu_irq = qemu_allocate_irqs(ls1a500_set_cpuirq, (void*)0, 8);
	cpu_irq1 = qemu_allocate_irqs(ls1a500_set_cpuirq, (void*)1, 8);

	/* init CPUs */

	cpu = MIPS_CPU(cpu_create(machine->cpu_type));
	env = &cpu->env;
	mycpu = env;

	cc = CPU_GET_CLASS(cpu);
	real_do_unassigned_access = cc->do_unassigned_access;
	cc->do_unassigned_access = mips_ls1a500_do_unassigned_access;

	reset_info[0] = g_malloc0(sizeof(ResetData));
	reset_info[0]->cpu = cpu;
	reset_info[0]->vector = env->active_tc.PC;
	qemu_register_reset(main_cpu_reset, reset_info[0]);

	/* Init CPU internal devices */
	cpu_mips_irq_init_cpu(cpu);
	cpu_mips_clock_init(cpu);
	
		/* allocate RAM */
	memory_region_init_ram(ram, NULL, "mips_r4k.ram", ram_size, &error_fatal);

        ALIAS_REGION_FROM_RAS_TO_RA(ram, 0, 0x10000000, address_space_mem, 0);
        ALIAS_REGION_FROM_RAS_TO_RA(ram, 0, 0x80000000, address_space_mem, 0x80000000);
	if(ram_size >= 0x40000000)
	{
		ALIAS_REGION_FROM_RAS_TO_RA(ram, ram_size - 0x20000000, 0x20000000, address_space_mem, 0x20000000);
	}

        //memory_region_init_iommu(iomem_root, NULL, &ls1a_pcidma_iommu_ops, "ls1a500 axi", UINT32_MAX);
        memory_region_init(iomem_root, NULL,  "ls1a500 axi", UINT64_MAX);
	address_space_init(as,iomem_root, "ls1a500 axi memory");
        ALIAS_REGION_FROM_RAS_TO_RA(ram, 0, 0x10000000, iomem_root, 0);
        ALIAS_REGION_FROM_RAS_TO_RA(ram, 0, 0x80000000, iomem_root, 0x80000000);

	//memory_region_init_io(iomem, &mips_qemu_ops, NULL, "mips-qemu", 0x10000);
	//memory_region_add_subregion(address_space_mem, 0x1fbf0000, iomem);

    if (kernel_filename) {
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        ((int64_t *)aui_boot_code)[1] = load_kernel(machine->dtb);
    }
   else ((int64_t *)aui_boot_code)[1] = env->active_tc.PC;
        ((int64_t *)aui_boot_code)[2] = 0;

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
    } else if ((flash_dinfo = drive_get(IF_PFLASH,0,0)))
	ddr2config = 1;
    {
        bios = g_new(MemoryRegion, 1);
        memory_region_init_ram(bios, NULL, "mips_r4k.bios0", BIOS_SIZE, &error_fatal);
        memory_region_set_readonly(bios, true);
        memory_region_add_subregion(get_system_memory(), 0x1e000000, bios);

	bios_size = sizeof(aui_boot_code);
	rom_add_blob_fixed("bios",aui_boot_code,bios_size,0x1e000000);
    }
  reset_info[0]->vector = (int)0xbe000000;
    if (filename) {
        g_free(filename);
    }


	/* Register 64 KB of IO space at 0x1f000000 */
	//isa_mmio_init(0x1ff00000, 0x00010000);
	//isa_mem_base = 0x10000000;
	pci_bus =pcibus_ls1a500_init(0, NULL,pci_ls1a500_map_irq, ram);
	ls1a500_irq =ls2k_intctl_init(get_system_memory(), 0x1Fe11400, cpu_irq, ls1a500_pci_bus->address_space_mem);
	ls1a500_irq1=ls2k_intctl_init(get_system_memory(), 0x1Fe11440, cpu_irq1, ls1a500_pci_bus->address_space_mem);


	if (serial_hd(0))
		serial_mm_init(address_space_mem, 0x1ff40000, 0,ls1a500_irq[0],115200,serial_hd(0), DEVICE_NATIVE_ENDIAN);

	if (serial_hd(1))
		serial_mm_init(address_space_mem, 0x1ff40400, 0,ls1a500_irq[1],115200,serial_hd(1), DEVICE_NATIVE_ENDIAN);

	if (serial_hd(2))
		serial_mm_init(address_space_mem, 0x1ff40800, 0,ls1a500_irq[2],115200,serial_hd(2), DEVICE_NATIVE_ENDIAN);

	if (serial_hd(3))
		serial_mm_init(address_space_mem, 0x1ff40c00, 0,ls1a500_irq[3],115200,serial_hd(3), DEVICE_NATIVE_ENDIAN);

	if (serial_hd(4))
		serial_mm_init(address_space_mem, 0x1ff41000, 0,ls1a500_irq1[30],115200,serial_hd(4), DEVICE_NATIVE_ENDIAN);

	if (serial_hd(5))
		serial_mm_init(address_space_mem, 0x1ff41400, 0,ls1a500_irq1[30],115200,serial_hd(5), DEVICE_NATIVE_ENDIAN);


#if 0
	sysbus_create_simple("ls1a500_acpi",0x1fe7c000, ls1a500_irq[0]);
#endif

	{
#if 1

	PCIDevice *pci_dev = pci_create_multifunction(pci_bus[1], -1, false, "e1000e");
	DeviceState *dev = DEVICE(pci_dev);
	if(nd_table[2].used)
		qdev_set_nic_properties(dev, &nd_table[2]);
	DeviceClass *dc = DEVICE_GET_CLASS(dev);
	PCIDeviceClass *k = PCI_DEVICE_CLASS(DEVICE_CLASS(dc));
	k->romfile = NULL;
	dc->vmsd = NULL;
	qdev_init_nofail(dev);


#endif
	}
#if 0
{
    PCIDevice *dev = pci_create_multifunction(pci_bus[0], -1, false, "pciram");
    qdev_prop_set_uint16(&dev->qdev, "vendor", 0x1002);
    qdev_prop_set_uint16(&dev->qdev, "device", 0x9615);
    qdev_prop_set_uint32(&dev->qdev, "bar0", (~(0x04000-1))|1);
    qdev_init_nofail(&dev->qdev);
}
#endif

#if 0
	{
		sysbus_create_simple("ls1a500_wdt", 0x1fe7c060, NULL);
	}
#endif

	{
		DeviceState *dev,*dev1;
		void *bus;
		qemu_irq cs_line;
		dev=sysbus_create_simple("ls1a_spi",0x1fd00000, ls1a500_irq[8]);
		bus = qdev_get_child_bus(dev, "ssi");
		if(flash_dinfo)
		{
			dev1 = ssi_create_slave_no_init(bus, "spi-flash");
			qdev_prop_set_drive(dev1, "drive", blk_by_legacy_dinfo(flash_dinfo), &error_fatal);
			qdev_prop_set_uint32(dev1, "size", 0x100000);
			qdev_prop_set_uint64(dev1, "addr", 0x1fc00000);
			qdev_init_nofail(dev1);
		cs_line = qdev_get_gpio_in_named(dev1, "ssi-gpio-cs",  0);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1 , cs_line);
		}
		//else dev1 = ssi_create_slave(bus, "ssi-sd");
		
    		DriveInfo *spinand = drive_get(IF_MTD, 0, 1);
		if (spinand)
		{
			dev1 = ssi_create_slave_no_init(bus, "spi-nand");
			qdev_prop_set_int32(dev1, "ftype", 0x2);
			qdev_init_nofail(dev1);
		cs_line = qdev_get_gpio_in_named(dev1, "ssi-gpio-cs",  0);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 2 , cs_line);
		}
	}

	{
    hwaddr flash_base   = 0x1a000000;
    size_t flash_sector_size        = 128 * KiB;
    size_t flash_size               = 64 * MiB;
    DriveInfo *dinfo = drive_get(IF_PFLASH, 0, 1);
    /* Spansion S29NS128P */
    liodev[0] = pflash_cfi02_register(0, NULL, "lioflash", flash_size,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                          flash_sector_size, flash_size / flash_sector_size,
                          1, 2, 0x89, 0x227e, 0x2248, 0x2201, 0x555, 0x2aa, 0);

        memory_region_del_subregion(get_system_memory(), liodev[0]->mmio[0].memory);
	flash_size               = 128 * MiB;

	dinfo = drive_get(IF_PFLASH, 0, 2);
    liodev[1] = pflash_cfi02_register(0, NULL, "lioflash1", flash_size,
                          dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                          flash_sector_size, flash_size / flash_sector_size,
                          1, 2, 0x1, 0x227e, 0x2248, 0x2201, 0x555, 0x2aa, 0);
        memory_region_del_subregion(get_system_memory(), liodev[1]->mmio[0].memory);

    liomr = ALIAS_REGION_FROM_RAS_TO_RA(liodev[0]->mmio[0].memory, ((gpio_ov>>44)&7)*0x2000000, 0x2000000, get_system_memory(), flash_base);
mips_qemu_writel (GPIO0_OUT_ADDR, 0, 0, 8);
	}



#if 0
	{
		DeviceState *dev;
		SysBusDevice *s;
		dev = qdev_create(NULL, "ls1a500_ac97");
		printf("ac97 dev=%p\n",dev);
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x1fe74000);
		sysbus_connect_irq(s, 0, ls1a500_irq[14]);
		sysbus_connect_irq(s, 1, ls1a500_irq[15]);
	}
#endif

	{
		DeviceState *dev;
		SysBusDevice *s;
		dev = qdev_create(NULL, "ls1a_dma");
		qdev_prop_set_uint8(dev, "mode", 0x1);
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x1fe10c00);
	}
	{
		DeviceState *dev;
		SysBusDevice *s;
		dev = qdev_create(NULL, "ls1a_nand");
		qdev_prop_set_uint8(dev, "cs", 0x2);
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x1ff58040);
		sysbus_connect_irq(s, 0, ls1a500_irq1[12]);
	}

	{
		DriveInfo *dinfo;
		dinfo = drive_get(IF_SD, 0, 0);
		if (!dinfo) {
			fprintf(stderr, "qemu: missing SecureDigital device\n");
			exit(1);
		}

		ls1gpa_mmci_init(address_space_mem, 0x1ff64000,
				blk_by_legacy_dinfo(dinfo),
				ls1a500_irq[31]
				);
	}

#if 1
	{
		DeviceState *dev;
		SysBusDevice *s;
		dev = qdev_create(NULL, "ls1a_rtc");
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x1ff6c100);
		sysbus_connect_irq(s, 0, ls1a500_irq[16]);
		sysbus_connect_irq(s, 1, ls1a500_irq[17]);
	}
#endif

	{
		DeviceState *dev;
		void *bus;
		dev=sysbus_create_simple("ls1a_i2c",0x1ff48000, ls1a500_irq[22]);
		bus = qdev_get_child_bus(dev, "i2c");
		i2c_create_slave(bus, "ds1338", 0x68);
	}

	{
		DeviceState *dev;
		void *bus;
		dev=sysbus_create_simple("ls1a_i2c",0x1ff48800, ls1a500_irq[23]);
		bus = qdev_get_child_bus(dev, "i2c");
		i2c_create_slave(bus, "ds1338", 0x68);
	}
#if 1
{
	DeviceState *dev;
	SysBusDevice *s;
	Object *obj;	
	Error *err = NULL;


	dev = qdev_create(NULL, "ls2k_can");
	obj = object_new("can-bus");
	object_property_add_child(object_get_objects_root(), "canbus0", obj, &err);
	object_property_parse(OBJECT(dev), "canbus0", "canbus", &err);
	s = SYS_BUS_DEVICE(dev);
	qdev_init_nofail(dev);
	sysbus_connect_irq(s, 0, ls1a500_irq[16]);
	sysbus_mmio_map(s, 0, 0x1ff44000);
}
#endif
{
	DeviceState *dev;
	SysBusDevice *s;
	Object *obj;	
	Error *err = NULL;


	dev = qdev_create(NULL, "ls2k_can");
	obj = object_new("can-bus");
	object_property_add_child(object_get_objects_root(), "canbus1", obj, &err);
	object_property_parse(OBJECT(dev), "canbus1", "canbus", &err);
	s = SYS_BUS_DEVICE(dev);
	qdev_init_nofail(dev);
	sysbus_connect_irq(s, 0, ls1a500_irq[17]);
	sysbus_mmio_map(s, 0, 0x1ff45000);
}

	{
        DeviceState *hpet = qdev_try_create(NULL, TYPE_HPET);
        if (hpet) {
            /* For pc-piix-*, hpet's intcap is always IRQ2. For pc-q35-1.7
             * and earlier, use IRQ2 for compat. Otherwise, use IRQ16~23,
             * IRQ8 and IRQ2.
             */
            uint8_t compat = object_property_get_uint(OBJECT(hpet),
                    HPET_INTCAP, NULL);
            if (!compat) {
                qdev_prop_set_uint32(hpet, HPET_INTCAP, 1);
            }
            qdev_init_nofail(hpet);
            sysbus_mmio_map(SYS_BUS_DEVICE(hpet), 0, 0x1ff68000);

            sysbus_connect_irq(SYS_BUS_DEVICE(hpet), 0, ls1a500_irq[21]);
        }
	}

		{
			DeviceState *dev;

			dev = qdev_create(NULL, "sysbus-synopgmac");
			if(nd_table[0].used)
				qdev_set_nic_properties(dev, &nd_table[0]);
			qdev_prop_set_ptr(dev, "as", as);
			qdev_prop_set_int32(dev, "enh_desc", 1);
			qdev_prop_set_uint32(dev, "version", 0xd137);
			qdev_prop_set_uint32(dev, "hwcap", 0x1b0d2fbf);
			qdev_init_nofail(dev);
			sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1f020000);
			sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls1a500_irq[12]);
		}
		{
			DeviceState *dev;

			dev = qdev_create(NULL, "sysbus-synopgmac");
    if(nd_table[1].used)
    qdev_set_nic_properties(dev, &nd_table[1]);
			qdev_prop_set_ptr(dev, "as", as);
			qdev_prop_set_int32(dev, "enh_desc", 1);
			qdev_prop_set_uint32(dev, "version", 0xd137);
			qdev_prop_set_uint32(dev, "hwcap", 0x1b0d2fbf);
			qdev_init_nofail(dev);
			sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1f030000);
			sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls1a500_irq[14]);
		}
	{
		DeviceState *dev;
		dev = qdev_create(NULL, "exynos4210-ehci-usb");
		qdev_prop_set_ptr(dev, "as", as);
		qdev_init_nofail(dev);
		sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1f050000);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls1a500_irq1[18]);
	}
	{
		DeviceState *dev;
		dev = qdev_create(NULL, "sysbus-ohci");
		qdev_prop_set_uint32(dev, "num-ports", 4);
		qdev_prop_set_uint64(dev, "dma-offset", 0);
		qdev_prop_set_ptr(dev, "as", as);
		qdev_init_nofail(dev);
		sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1f058000);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls1a500_irq1[19]);

	}
	{
		DeviceState *dev;
		BusState *idebus[4];
		DriveInfo *hd;
		dev = qdev_create(NULL, "sysbus-ahci");
		qdev_prop_set_uint32(dev, "num-ports", 1);
		qdev_prop_set_ptr(dev, "as", as);
		qdev_init_nofail(dev);
		sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, 0x1f040000);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls1a500_irq[19]);
		hd=drive_get_next(IF_IDE);
		if(hd)
		{
			idebus[0] = qdev_get_child_bus(dev, "ide.0");

			dev = qdev_create(idebus[0], hd->media_cd ? "ide-cd" : "ide-hd");
			qdev_prop_set_uint32(dev, "unit", 0);
			qdev_prop_set_drive(dev, "drive", blk_by_legacy_dinfo(hd),&error_fatal);
			qdev_init_nofail(dev);

        		//ide_create_drive(&ahci->dev[i].port, 0, hd[i]);
		}
	}

	{

                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fe104b0, "0x1fe104b0", 0x4);
                memory_region_add_subregion(address_space_mem, 0x1fe104b0, iomem);
                iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fe104c0, "0x1fe104c0", 0x4);
                memory_region_add_subregion(address_space_mem, 0x1fe104c0, iomem);
                iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fe10c30, "0x1fe10c30", 0x4);
                memory_region_add_subregion(address_space_mem, 0x1fe10c30, iomem);

                iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fe10480, "0x1fe10480", 0x20);
                memory_region_add_subregion(address_space_mem, 0x1fe10480, iomem);

                iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fe10490, "0x1fe10490", 0x10);
                memory_region_add_subregion(address_space_mem, 0x1fe10490, iomem);

                iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fe104a0, "0x1fe104a0", 0x4);
                memory_region_add_subregion(address_space_mem, 0x1fe104a0, iomem);

                iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fe10100, "0x1fe10100", 0x4);
                memory_region_add_subregion(address_space_mem, 0x1fe10100, iomem);
#ifdef DEBUG_PCIEDMA
                iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fef0000, "0x1fef0000", 0x8);
                memory_region_add_subregion(address_space_mem, 0x1fef0000, iomem);
#endif
	}


	{
                ddrcfg_iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(ddrcfg_iomem, NULL, &mips_qemu_ops, (void *)0x0ff00000, "ddr", 0x100000);

		if(ddr2config)
			mips_qemu_writel (0, 0x1fe10100, 0x000, 4);
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
		sysbus_connect_irq(s, 0, ls1a500_irq1[25]);
		hdabus = QLIST_FIRST(&dev->child_bus);
		codec = qdev_create(hdabus, "hda-duplex");
		qdev_init_nofail(codec);
	}
#endif
	{
		MemoryRegion *iomem = g_new(MemoryRegion, 1);
		memory_region_init_ram(iomem, NULL, "ls1a500.gpio", 0x40, &error_fatal);
		memory_region_add_subregion(get_system_memory(), GPIO0_ADDR, iomem);
		SIMPLE_OPS(GPIO0_OUT_ADDR,8);
	}

		SIMPLE_OPS(0x1fe11040,16);
		SIMPLE_OPS(0x1fe10400,0x28);
	mypc_callback =  mypc_callback_for_net;
}


static void mips_machine_init(MachineClass *mc)
{
    mc->desc = "mips ls1a500 platform";
    mc->init = mips_ls1a500_init;
    mc->max_cpus = 1;
    mc->block_default_type = IF_IDE;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("loongson1a500");
}

DEFINE_MACHINE("ls1a500", mips_machine_init)

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

#define MAX_PILS 16

#include "ls2k_int.c"
//-------------------------
// pci bridge
//-----------------

static int pci_ls1a500_map_irq(PCIDevice *d, int pin)
{
	int dev=(d->devfn>>3)&0x1f;
	int fn=d->devfn& 7;

	if(pci_get_bus(d) != ls1a500_pci_bus)
		return pin;

	  switch(dev)
	  {
		case 5:
		/*GPU*/
		 return 29;
		break;

		case 7:
		/*xhci*/
		 return 30;
		break;

		case 9:
		/*PCIE PORT 0*/
		 return 32;;
		break;

		case 10:
		/*PCIE PORT 1*/
		 return 33;
		break;
	  }
	
	return  0;
}

static void pci_ls1a500_set_irq(void *opaque, int irq_num, int level)
{
	if(irq_num<32)
		qemu_set_irq(ls1a500_irq[irq_num],level);
	else 
		qemu_set_irq(ls1a500_irq1[irq_num-32],level);
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


#define TYPE_BONITO_PCI_HOST_BRIDGE "ls1a500-pcihost"
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
    PCIExpressHost parent_obj;
    PCIBus *bus;
    qemu_irq *pic;
    PCIBonitoState *pci_dev;
    MemoryRegion iomem_mem;
    MemoryRegion iomem_io;
    AddressSpace as_mem;
    AddressSpace as_io;
    MemoryRegion data_mem;
    MemoryRegion data_mem1;
    Notifier wakeup;
   int (*pci_map_irq)(PCIDevice *d, int irq_num);
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

static void bonito_initfn(PCIDevice *dev, Error **errp)
{
    PCIBonitoState *s = DO_UPCAST(PCIBonitoState, parent_obj.parent_obj, dev);
    SysBusDevice *sysbus = SYS_BUS_DEVICE(s->pcihost);

    pci_bridge_initfn(dev, TYPE_PCI_BUS);

    /* set the north bridge pci configure  mapping */
    memory_region_init_io(&s->conf_mem, NULL, &bonito_pciconf_ops, s,
                          "north-bridge-pci-config", 0x100);
    sysbus_init_mmio(sysbus, &s->conf_mem);


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

}


static void bonito_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = bonito_initfn;
    k->exit = pci_bridge_exitfn;
    k->vendor_id = 0x0014;
    k->device_id = 0x1a05;
    k->revision = 0x01;
    k->class_id = PCI_CLASS_BRIDGE_HOST;
    k->is_bridge = 1;
    k->config_write = pci_bridge_write_config;
    dc->reset = pci_bridge_reset;
    dc->desc = "Host bridge";
//pci_bridge_dev_initfn
}

static const TypeInfo bonito_info = {
    .name          = "LS1A500_Bonito",
    .parent        = TYPE_PCI_BRIDGE,
    .instance_size = sizeof(PCIBonitoState),
    .class_init    = bonito_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static AddressSpace *pci_dma_context_fn(PCIBus *bus, void *opaque, int devfn);

static void pcihost_wakeup_notifier(Notifier *notifier, void *data)
{
	BonitoState *pcihost = container_of(notifier, BonitoState, wakeup);
    WakeupReason *reason = data;
#define BDFR(b,d,f,r) ((b<<16)|(d<<11)|(f<<8)|r)
	pci_data_write(pcihost->bus,  BDFR(0,7,0,0x10), 0x1f060000, 4);
	pci_data_write(pcihost->bus,  BDFR(0,7,0,0x4), 0x7, 4);
}
#define MAX_SATA_PORTS     6
static PCIBus **pcibus_ls1a500_init(int busno, qemu_irq *pic, int (*board_map_irq)(PCIDevice *d, int irq_num), MemoryRegion *ram)
{
    DeviceState *dev;
    BonitoState *pcihost;
    PCIBonitoState *s;
    PCIDevice *d;
    SysBusDevice *sysbus;
    PCIBridge *br;
    PCIBus *bus2;
    DriveInfo *hd[MAX_SATA_PORTS];
    static PCIBus *pci_bus[4];
    int i;

    dev = qdev_create(NULL, TYPE_BONITO_PCI_HOST_BRIDGE);
    pcihost = BONITO_PCI_HOST_BRIDGE(dev);
    pcihost->pic = pic;
    pcihost->pci_map_irq = board_map_irq;
    qdev_init_nofail(dev);

    /* set the pcihost pointer before bonito_initfn is called */
    //d = pci_create(phb->bus, PCI_DEVFN(0, 0), "LS1A500_Bonito");
    for (i=0;i<2;i++)
    {
	char buf[16];
    d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(i, 0), true, "LS1A500_Bonito");
    sprintf(buf, "pcie-%d.0", i);
    qdev_set_id(DEVICE(d), g_strdup(buf));

    s = DO_UPCAST(PCIBonitoState, parent_obj.parent_obj, d);
    s->pcihost = pcihost;
    pcihost->pci_dev = s;
    br = PCI_BRIDGE(d);
    pci_bridge_map_irq(br, "Advanced PCI Bus secondary bridge 1", board_map_irq);
    qdev_init_nofail(DEVICE(d));
    bus2 = pci_bridge_get_sec_bus(br);

    pci_setup_iommu(bus2, pci_dma_context_fn, pcihost);
    pci_bus[i] = bus2;
    }


{
	DeviceState *dev;
	dev = sysbus_create_simple("ls2h_fb", 0x1f010000, NULL);
	qdev_prop_set_ptr(dev, "root", &pcihost->iomem_mem);
}


    d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(4, 0), true, "pciram");
    qdev_prop_set_uint32(DEVICE(d), "bar0", ~(0x00001000-1));
    qdev_init_nofail(DEVICE(d));
    pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
    pci_set_word(d->config + PCI_DEVICE_ID, 0x7a04);


    {
	    //gpu
	    MemoryRegion *iomem = g_new(MemoryRegion, 1);
	    memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)GPUBASE, "gpu", 0x8000);
	    d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(5, 0), true, "pciram");
	    qdev_prop_set_uint32(DEVICE(d), "bar0", ~(0x00008000-1)|4);
	    qdev_prop_set_ptr(DEVICE(d), "iomem0", iomem);
	    
	//qdev_prop_set_uint32(DEVICE(d), "bar1", ~(0x01000000-1)|4);
	  //  qdev_prop_set_uint32(DEVICE(d), "bar2", ~(0x00000100-1)|4);
	    qdev_init_nofail(DEVICE(d));
	    pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
	    pci_set_word(d->config + PCI_DEVICE_ID, 0x7a15);
    }

    //pci_create_simple(pcihost->bus, PCI_DEVFN(7,0), "nec-usb-xhci");
    {
	MemoryRegion *address_space_mem = get_system_memory();
	    PCIDevice *dev = pci_create_multifunction(pcihost->bus, PCI_DEVFN(7,0), false, "nec-usb-xhci");
	    qdev_init_nofail(&dev->qdev);
	    pci_set_word(dev->config + PCI_VENDOR_ID, 0xffff);
	    pci_set_word(dev->config + PCI_DEVICE_ID, 0xffff);
    pci_set_word(dev->config + PCI_COMMAND,
                 PCI_COMMAND_MASTER|PCI_COMMAND_MEMORY|PCI_COMMAND_IO);
    pci_set_word(dev->wmask + PCI_COMMAND, 0);
//should set after reset
    pcihost->wakeup.notify = pcihost_wakeup_notifier;
    qemu_register_wakeup_notifier(&pcihost->wakeup);
   SIMPLE_OPS(0x1f06c100,0x400);
    }

    sysbus = SYS_BUS_DEVICE(pcihost);
     /*devices header*/
    sysbus_mmio_map(sysbus, 0, 0x16800000);
    sysbus_mmio_map(sysbus, 1, 0xfe00000000ULL);

    
    ALIAS_REGION_FROM_RAS_TO_RA(&pcihost->iomem_mem, 0x1f060000, 0x10000, get_system_memory(), 0x1f060000);
    ALIAS_REGION_FROM_RAS_TO_RA(&pcihost->iomem_mem, 0x10000000UL,  0x02000000UL, get_system_memory(), 0x10000000UL);
    ALIAS_REGION_FROM_RAS_TO_RA(&pcihost->iomem_mem, 0x40000000UL,  0x20000000UL, get_system_memory(), 0x40000000UL);
    memory_region_add_subregion(get_system_memory(), 0x16400000UL, &pcihost->iomem_io);

    ALIAS_REGION_FROM_RAS_TO_RA(ram, 0, 0x80000000, &pcihost->iomem_mem, 0x80000000);

    ALIAS_REGION_FROM_RAS_TO_RA(ram, 0, 0x10000000, &pcihost->iomem_mem, 0);
    if(memory_region_size(ram) >= 0x40000000)
    {
	    ALIAS_REGION_FROM_RAS_TO_RA(ram, memory_region_size(ram) - 0x20000000, 0x20000000, &pcihost->iomem_mem, 0x20000000);
    }

    ls1a500_pci_bus = pcihost->bus;
//pcihost
//pci_ls1a500_config_writel

    return pci_bus;
}


static AddressSpace *pci_dma_context_fn(PCIBus *bus, void *opaque, int devfn)
{
    BonitoState *pcihost = opaque;
    return &pcihost->as_mem;
}

static void pci_ls1a500_config_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	BonitoState *phb = opaque;

	addr &= 0xffffff;

	pci_data_write(phb->bus,  addr, val, size);
}

static uint64_t pci_ls1a500_config_readl (void *opaque, hwaddr addr, unsigned size)
{
	BonitoState *phb = opaque;
	uint32_t val;

	addr &= 0xffffff;


	val = pci_data_read(phb->bus, addr, size);
	//printf("pci_ls1a500_config_readl 0x%x 0x%x\n", (int)addr, val);
	return val;
}


static const MemoryRegionOps pci_ls1a500_config_ops = {
    .read = pci_ls1a500_config_readl,
    .write = pci_ls1a500_config_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


/*
two way to translate pci dma address:
pci_setup_iommu
memory_region_init_iommu
pci_setup_iommu will not change addr.
memory_region_init_iommu can translate region and addr.
*/

static void bonito_pcihost_initfn(DeviceState *dev, Error **errp)
{
    BonitoState *pcihost;
    PCIHostState *phb;
    SysBusDevice *sysbus;
    pcihost = BONITO_PCI_HOST_BRIDGE(dev);
    sysbus = SYS_BUS_DEVICE(pcihost);

    memory_region_init(&pcihost->iomem_mem, OBJECT(pcihost), "system", INT64_MAX);
    address_space_init(&pcihost->as_mem, &pcihost->iomem_mem, "pcie memory");
#ifdef DEBUG_PCIEDMA
    ls1a500_pci_as = &pcihost->as_mem;
#endif

    /* Host memory as seen from the PCI side, via the IOMMU.  */

    memory_region_init(&pcihost->iomem_io, OBJECT(pcihost), "system", 0x10000);
    address_space_init(&pcihost->as_io, &pcihost->iomem_io, "pcie io");

    phb = PCI_HOST_BRIDGE(dev);
    pcihost->bus = phb->bus = pci_register_root_bus(DEVICE(dev), "pci",
                                pci_ls1a500_set_irq, pcihost->pci_map_irq, pcihost->pic,
                                &pcihost->iomem_mem, &pcihost->iomem_io,
                                PCI_DEVFN(0, 0), 64, TYPE_PCIE_BUS);


    pci_setup_iommu(pcihost->bus, pci_dma_context_fn, pcihost);

    /* set the south bridge pci configure  mapping */
    memory_region_init_io(&pcihost->data_mem, NULL, &pci_ls1a500_config_ops, pcihost,
                          "south-bridge-pci-config", 0x800000);
    sysbus_init_mmio(sysbus, &pcihost->data_mem);

    memory_region_init_io(&pcihost->data_mem1, NULL, &pci_ls1a500_config_ops, pcihost,
                          "south-bridge-pci-config", 0x200000000);
    sysbus_init_mmio(sysbus, &pcihost->data_mem1);
}

static const char *ls1a500_host_root_bus_path(PCIHostState *host_bridge,
                                          PCIBus *rootbus)
{
    return "0000:00";
}

static void bonito_pcihost_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIHostBridgeClass *hc = PCI_HOST_BRIDGE_CLASS(klass);

    hc->root_bus_path = ls1a500_host_root_bus_path;
    dc->realize = bonito_pcihost_initfn;
}

static const TypeInfo bonito_pcihost_info = {
    .name          = TYPE_BONITO_PCI_HOST_BRIDGE,
    .parent        = TYPE_PCIE_HOST_BRIDGE,
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
#define LOONGSON_2K
#include "loongson_bootparam.c"
