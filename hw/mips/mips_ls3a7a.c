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
#include "loongson3a_rom.h"
#include "hw/timer/hpet.h"
#include "ls7a_int.h"
#include "sysemu/device_tree.h"
#include "libfdt.h"

static LS7A_INTCTLState *ls7a;
extern target_ulong mypc;

#define PHYS_TO_VIRT(x) ((x) | ~(target_ulong)0x7fffffff)

#define VIRT_TO_PHYS_ADDEND (-((int64_t)(int32_t)0x80000000))

#define MAX_IDE_BUS 2
#define TARGET_REALPAGE_MASK (TARGET_PAGE_MASK<<2)

static const int ide_iobase[2] = { 0x1f0, 0x170 };
static const int ide_iobase2[2] = { 0x3f6, 0x376 };
static const int ide_irq[2] = { 14, 15 };
static qemu_irq *ls3a7a_irq;

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
static PCIBus *ls3a7a_pci_bus;
#define align(x) (((x)+15)&~15)
static int pci_ls3a7a_map_irq(PCIDevice *d, int irq_num);


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

#define _str(x) #x
#define str(x) _str(x)
#define SIMPLE_OPS(ADDR,SIZE) \
	({\
		MemoryRegion *address_space_mem = get_system_memory(); \
                MemoryRegion *iomem = g_new(MemoryRegion, 1);\
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)ADDR, str(ADDR) , SIZE);\
                memory_region_add_subregion_overlap(address_space_mem, ADDR, iomem, 1);\
		iomem;\
	})

#define ALIAS_REGION(ADDR,SIZE,ALIAS) \
({\
    MemoryRegion *address_space_mem = get_system_memory(); \
    MemoryRegion *alias_mr = g_new(MemoryRegion, 1); \
    memory_region_init_alias(alias_mr, NULL, NULL, address_space_mem, ADDR, SIZE); \
    memory_region_add_subregion(address_space_mem, ALIAS, alias_mr); \
})


#define CONFBASE 0xc0000000
#define GPUBASE 0xd0000000
static MemoryRegion *ddrcfg_iomem;
static MemoryRegion *iomem160;
static int reg180;
static int reg424;
static unsigned char mem200[256];
static MemoryRegion *cachelock_iomem[4];
static uint64_t cachelock_base[4];
static uint64_t cachelock_size[4];

static char ddr_level_reg[0x1000];
static void mips_qemu_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	int i, j;
	unsigned long long base;
	unsigned long long mask, bsize;
	unsigned int old;
	hwaddr offset = addr;
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case 0x0ff00000 ... 0x0ff01000-1:
		case 0x40000000 ... 0x40001000-1:
		 memcpy(ddr_level_reg + offset, &val, size);
		break;
		case 0x3ff00200 ... 0x3ff002ff:
		old = *(unsigned int *)(mem200+addr-0x3ff00200);
		memcpy(mem200 + addr-0x3ff00200, &val, size);
		 for(i = 0,j = 0;i < 0x20;i += 8, j += 1)
		 {
			
			if(addr == 0x3ff00200+i+4 && ((old ^ val) & 0x80000000))
			{ 
				base = *(unsigned long long *)(mem200+i) & 0x7fffffffffffffffULL;
				mask = *(unsigned long long *)(mem200+i+0x40);
				bsize = ~mask + 1;

				memory_region_transaction_begin();
				if(val & 0x80000000)
				{
					char *buf = NULL;
					printf("cachelock base = 0x%llx size = 0x%llx\n", base, bsize);
					if (!cachelock_iomem[j]) {
						buf = g_new(char, bsize);
						address_space_read(&address_space_memory, base, MEMTXATTRS_UNSPECIFIED, buf, bsize);
						cachelock_iomem[j] = g_new(MemoryRegion, 1);
						memory_region_init_ram_nomigrate(cachelock_iomem[j], NULL, "mips_r4k.cachemem", bsize, &error_fatal);
						cachelock_base[j] = base;
						cachelock_size[j] = bsize;
					}
					memory_region_add_subregion_overlap(get_system_memory(), base, cachelock_iomem[j], 1);
					memory_region_transaction_commit();
					if (buf) {
						address_space_write(&address_space_memory, base, MEMTXATTRS_UNSPECIFIED, buf, bsize);
						g_free(buf);
					}
				}
				else if(cachelock_iomem[j])
				{
					char *buf = NULL;
					base = cachelock_base[j];
					size = cachelock_size[j];
					printf("uncachelock base = 0x%llx size = 0x%llx\n", base, bsize);
					buf = g_new(char, bsize);
					address_space_read(&address_space_memory, base, MEMTXATTRS_UNSPECIFIED, buf, bsize);
					memory_region_del_subregion(get_system_memory(), cachelock_iomem[j]);
					memory_region_unref(cachelock_iomem[j]);
					memory_region_transaction_commit();
					address_space_write(&address_space_memory, base, MEMTXATTRS_UNSPECIFIED, buf, bsize);
					g_free(buf);
					g_free(cachelock_iomem[j]);
					cachelock_iomem[j] = NULL;
				}

			}
		}
		break;
		case 0x1fe00180:
			reg180 = val;
			memory_region_transaction_begin();
			if(ddrcfg_iomem->container == get_system_memory())
				memory_region_del_subregion(get_system_memory(), ddrcfg_iomem);

			if((val&0x10) != 0x10)
			{
				memory_region_add_subregion_overlap(get_system_memory(), 0x0ff00000, ddrcfg_iomem, 1);
			}

			memory_region_transaction_commit();
			break;
		case 0x10010424:
			reg424 = val;
			memory_region_transaction_begin();
			if(iomem160->container == get_system_memory())
				memory_region_del_subregion(get_system_memory(), iomem160);

			if((val&0x10000000) != 0x10000000)
			{
				memory_region_add_subregion_overlap(get_system_memory(), 0x40000000, iomem160, 1);
			}

			memory_region_transaction_commit();
			break;
	}
}

static uint64_t mips_qemu_readl (void *opaque, hwaddr addr, unsigned size)
{
	uint64_t val;
	hwaddr offset = addr;
	addr=((hwaddr)(long)opaque) + addr;
//	printf("%s 0x%llx\n", __FUNCTION__, (long long)addr);
	switch(addr)
	{
		case 0x3ff00200 ... 0x3ff002ff:
		 val = 0;
		 memcpy(&val, mem200 + addr-0x3ff00200, size);
		 return val;
		break;
		case 0x1fe001b0 ... 0x1fe001b7:
		{
		uint64_t d = 0x0481041204110C85ULL;
		val = 0;
		memcpy(&val, (char *)&d + addr - 0x1fe001b0, size);
		}
		return val;
		break;
		case 0x1fe00180:
		return reg180;
		case CONFBASE+0x4b0+4:
		return random();
		case CONFBASE+0x4c0+4:
		return random();
		case GPUBASE+4:
		case GPUBASE+0:
		case GPUBASE+0x100:
		case 0xefdfb000044:
		return random();
		case 0xefdfb000178:
		return 8;
		case 0xefdfe0001f4:
		return 8;
		case 0xefdfe000044:
		return 0x40;
		case 0x10013ff8:
		return 0x7a000000;
		case 0x10010484:
		case 0x10010494:
		case 0x100104a4:
		case 0x100104b4:
		case 0x100104c4:
		return 0x80;
		case 0x10010424:
		return 0xfff00;
		case 0x10010594:
		case 0x10010694:
		case 0x10010794:
		case 0x10010894:
		case 0x100105b4:
		case 0x100106b4:
		case 0x100107b4:
		case 0x100108b4:
		return 4;
		case 0x100105d4:
		case 0x100106d4:
		case 0x100107d4:
		case 0x100108d4:
		case 0x100105dc:
		case 0x100106dc:
		case 0x100107dc:
		case 0x100108dc:
		case 0x100105f4:
		case 0x100106f4:
		case 0x100107f4:
		case 0x100108f4:
		case 0x100105fc:
		case 0x100106fc:
		case 0x100107fc:
		case 0x100108fc:

		case 0x10010614:
		case 0x10010714:
		case 0x10010814:
		case 0x10010914:
		case 0x1001061c:
		case 0x1001071c:
		case 0x1001081c:
		case 0x1001091c:
		return 4;
		case 0xe0040000160:
		return 1<<24;
		case 0x1fe00100:
		return 0x10000;
		case 0x1fe001c0:
		return 0x0909017b;
		case 0x0ff00000 ... 0x0ff01000 - 1:
		case 0x40000000 ... 0x40001000 - 1:
		if (offset >= 0x160 && offset <=0x167) {
			uint64_t data = 0x000000000f000101;
			val = 0;
			memcpy(&val, (char *)&data + (offset - 0x160), size);
			return val;
		}
		else if (offset == 0x181 || offset == 0x185 || offset == 0x186) /*181:req,185:ready,186:done*/
			return 1;
		else if (offset >= 0x187 && offset <= 0x18f) {
			int slice = offset - 0x187;
			/*180:mode, 187,188:response*/
			//wlvl
			if (ddr_level_reg[0x180] == 1)
			return ddr_level_reg[0x3a+slice*0x20] >= 1 && ddr_level_reg[0x3a+slice*0x20] < 20? 0x1:0;
			//glvl
			else if (ddr_level_reg[0x180] == 2)
			return ddr_level_reg[0x38+slice*0x20] >= 1 && ddr_level_reg[0x38+slice*0x20] < 20? 0x3:0;
			else
			return 0;
		} else {
			val = 0;	
			memcpy(&val, ddr_level_reg + offset, size);
			return val;
		}
		default:
		return random();
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
		ret +=1+snprintf(params_buf+ret,PBUF_SIZE-ret, "rd_start=0x" TARGET_FMT_lx " rd_size=%li %s",
				PHYS_TO_VIRT((uint32_t)initrd_offset),
				initrd_size, loaderparams.kernel_cmdline);
	} else {
		ret +=1+snprintf(params_buf+ret, PBUF_SIZE-ret, "%s", loaderparams.kernel_cmdline);
	}
	//argv2
	*parg_env++=0;

	//env
	sprintf(memenv,"memsize=%d",(int)(loaderparams.ram_size>=0xf000000?240:(loaderparams.ram_size>>20)));
	sprintf(highmemenv,"highmemsize=%d",(int)(loaderparams.ram_size>0x10000000?(loaderparams.ram_size>>20)-256:0));


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

	/* Store command line.  */
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
	loaderparams.a2 = (target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR + ret;
        printf("env %x\n", BOOTPARAM_PHYADDR + ret);

	if(dtb)
	{
	int size;
	void *fdt;
	int err;
        uint64_t ram_low_sz, ram_high_sz;
	struct boot_params *bp = (void *)boot_params_buf;
	struct system_loongson *s = (void *)&bp->efi.smbios.lp + bp->efi.smbios.lp.system_offset; 

        ret = boot_params_p - params_buf;
	s->of_dtb_addr = (target_ulong)0xffffffff80000000ULL+BOOTPARAM_PHYADDR + ret;

	fdt = load_device_tree(dtb, &size);

        params_size = ret + size + 8;
        params_buf = g_realloc(params_buf, params_size);
        memcpy(params_buf + ret, fdt, size);

	qemu_fdt_dumpdtb(fdt, size);
	}
	rom_add_blob_fixed("params", params_buf, params_size,
			BOOTPARAM_PHYADDR);
return 0;
}

static uint64_t cpu_mips_kseg0_to_phys1(void *opaque, uint64_t addr)
{
    return (addr & 0x1fffffffll) + (unsigned long long)opaque;
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
	char *s;
	unsigned long long offset;
	s = getenv("LOADOFFSET");
	offset = s?strtoull(s, 0, 0) : 0;
	kernel_size = load_elf(loaderparams.kernel_filename, cpu_mips_kseg0_to_phys1, (void*)offset,
                           (uint64_t *)&entry, (uint64_t *)&kernel_low,
                           (uint64_t *)&kernel_high,0,EM_MIPS, 1, 0);
    if (kernel_size >= 0) {
        if ((entry & ~0x7fffffffULL) == 0x80000000)
            entry = (int32_t)entry;
	if (offset)
	    entry = cpu_mips_kseg0_to_phys1((void*)offset, entry) + 0x9800000000000000ULL;
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
#if TARGET_LONG_SIZE == 8
	if ((unsigned long long)env->active_tc.PC < 0xFFFFFFFF80000000ULL)
		env->CP0_Status |= 0xe0;
#endif
}

static int uart_irqstatus = 0;
static int pci_irqstatus = 0;
static void ls3auartpci_set_irq(void *opaque, int irq, int level)
{
	CPUMIPSState *env = opaque;
	if (irq < 2) {
	if(level) uart_irqstatus |= 1<<irq;
	else uart_irqstatus &= ~(1<<irq);
	} else {
	if(level) pci_irqstatus |= 1<<(irq - 2);
	else pci_irqstatus &= ~(1<<(irq - 2));
	}
	qemu_set_irq(env->irq[2],(!!uart_irqstatus) | (!!pci_irqstatus));

}

static void ls7auart_set_irq(void *opaque, int irq, int level)
{
	qemu_irq  ls7airq = opaque;
	static int ls7auart_irqstatus = 0;
	if(level) ls7auart_irqstatus |= 1<<irq;
	else ls7auart_irqstatus &= ~(1<<irq);

	qemu_set_irq(ls7airq, !!ls7auart_irqstatus);
}

#define MAX_CPUS 4
static CPUMIPSState *mycpu[MAX_CPUS];

#define CORE0_STATUS_OFF       0x000
#define CORE0_EN_OFF           0x004
#define CORE0_SET_OFF          0x008
#define CORE0_CLEAR_OFF        0x00c
#define CORE0_BUF_20           0x020
#define CORE0_BUF_28           0x028
#define CORE0_BUF_30           0x030
#define CORE0_BUF_38           0x038


#define MYID 0xa0
//#define DEBUG_LS2K
#ifdef DEBUG_LS2K
#define IPI_DPRINTF printf
#else
#define IPI_DPRINTF(...)
#endif

typedef struct gipi_single {
    uint32_t status;
    uint32_t en;
    uint32_t set;
    uint32_t clear;
    uint32_t buf[8];
    qemu_irq irq;
} gipi_single;

typedef struct gipiState  gipiState;
struct gipiState {
	gipi_single core[8];
} ;


static void gipi_writel(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
	gipiState * s = opaque;
//	CPUState *cpu = current_cpu;
	int no = (addr>>8)&3;


	if(size!=4) hw_error("size not 4");

	//    printf("gipi_writel addr=%llx val=%8x\n", addr, val);
	addr &= 0xff;
	switch(addr){
		case CORE0_STATUS_OFF: 
			hw_error("CORE0_SET_OFF Can't be write\n");
			break;
		case CORE0_EN_OFF:
			//if((cpu->mem_io_vaddr&0xff)!=addr) break;
			s->core[no].en = val;
			break;
		case CORE0_SET_OFF:
			s->core[no].status |= val;
            if (s->core[no].status != 0) {
			qemu_irq_raise(s->core[no].irq);
            }
			break;
		case CORE0_CLEAR_OFF:
			//if((cpu->mem_io_vaddr&0xff)!=addr) break;
			s->core[no].status ^= val;
            if (s->core[no].status == 0) {
			qemu_irq_lower(s->core[no].irq);
            }
			break;
		case 0x20 ... 0x3c:
			s->core[no].buf[(addr-0x20)/4] = val;
			break;
		default:
			break;
	}
	IPI_DPRINTF("gipi_write: addr=0x%02x val=0x%02llx cpu=%d pc=%llx\n", (int)addr, (long long)val, (int)cpu->cpu_index, (long long)mypc);
}

static uint64_t gipi_readl(void *opaque, hwaddr addr, unsigned size)
{
	gipiState * s = opaque;
#ifdef DEBUG_LS2K
	CPUState *cpu = current_cpu;
#endif
	uint64_t ret=0;
	int no = (addr>>8)&3;

	addr &= 0xff;
	if(size!=4) hw_error("size not 4 %d", size);

	switch(addr){
		case CORE0_STATUS_OFF: 
			ret =  s->core[no].status;
			break;
		case CORE0_EN_OFF:
			ret =  s->core[no].en;
			break;
		case CORE0_SET_OFF:
			ret = 0;//hw_error("CORE0_SET_OFF Can't be Read\n");
			break;
		case CORE0_CLEAR_OFF:
			ret = 0;//hw_error("CORE0_CLEAR_OFF Can't be Read\n");
		case 0x20 ... 0x3c:
			ret = s->core[no].buf[(addr-0x20)/4];
			break;
		default:
			break;
	}

	IPI_DPRINTF("gipi_read: addr=0x%02x val=0x%02llx cpu=%d pc=%llx\n", (int)addr, (long long)ret, (int)cpu->cpu_index, (long long)mypc);
	return ret;
}



static const MemoryRegionOps gipi_ops = {
    .read = gipi_readl,
    .write = gipi_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};



static MemoryRegion *gipi_iomem;

static int godson_ipi_init(qemu_irq parent_irq , unsigned long index, gipiState * s)
{
  int mysize=0x1000;
  s->core[index].irq = parent_irq;
  if((index==0)||(index==4)) {
      memory_region_add_subregion(get_system_memory(), (0x3ff01000|((unsigned long long)(index/4) << 44)) + (index % 4)*mysize, gipi_iomem);
  }
      return 0;
}


static const int sector_len = 32 * 1024;

static PCIBus **pcibus_ls3a7a_init(int busno,qemu_irq *pic, int (*board_map_irq)(PCIDevice *d, int irq_num), MemoryRegion *ram, MemoryRegion *ram1);




static CPUUnassignedAccess real_do_unassigned_access;
static void mips_ls3a7a_do_unassigned_access(CPUState *cpu, hwaddr addr,
                                           bool is_write, bool is_exec,
                                           int opaque, unsigned size)
{
    if (!is_exec) {
        /* ignore invalid access (ie do not raise exception) */
        return;
    }
    (*real_do_unassigned_access)(cpu, addr, is_write, is_exec, opaque, size);
}

struct hpet_fw_config hpet_cfg = {.count = UINT8_MAX};

static DriveInfo *flash_dinfo=NULL;
static unsigned long *cpu_irqset_bitmap; 
static unsigned long *cpu_irqclr_bitmap; 
static void ht_set_irq(void *opaque, int irq, int level);
static qemu_irq *ls3a_intctl_init(MemoryRegion *iomem_root, CPUMIPSState *env[]);
static void mips_ls3a7a_init(MachineState *machine)
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
	qemu_irq *ht_irq, *ls3auartpci_irq, *ls7auart_irq;
	PCIBus *pci_bus, **ppci_bus;
	CPUClass *cc;
	MemoryRegion *iomem_root = g_new(MemoryRegion, 1);
	AddressSpace *as = g_new(AddressSpace, 1);
	int i;

	gipiState * gipis =g_malloc0(sizeof(gipiState));
	gipi_iomem = g_new(MemoryRegion, 1);
	memory_region_init_io(gipi_iomem, NULL, &gipi_ops, (void *)gipis, "gipi", 0x400);

	/* init CPUs */

	for(i = 0; i < smp_cpus; i++) {
	cpu = MIPS_CPU(cpu_create(machine->cpu_type));
		env = &cpu->env;
		env->CP0_EBase |= i;
		mycpu[i] = env;

		cc = CPU_GET_CLASS(cpu);
		real_do_unassigned_access = cc->do_unassigned_access;
		cc->do_unassigned_access = mips_ls3a7a_do_unassigned_access;

		reset_info[i] = g_malloc0(sizeof(ResetData));
		reset_info[i]->cpu = cpu;
		reset_info[i]->vector = env->active_tc.PC;
		qemu_register_reset(main_cpu_reset, reset_info[i]);

	/* Init CPU internal devices */
		cpu_mips_irq_init_cpu(cpu);
		cpu_mips_clock_init(cpu);
		godson_ipi_init(env->irq[6] , i, gipis);  // by zxh&dw
	}
	
	env = mycpu[0];

	MemoryRegion *htmem = g_new(MemoryRegion, 1);
	memory_region_init_alias(htmem, NULL, "lowpcimem", address_space_mem, 0x10000000, 0x10000000ULL);
	MemoryRegion *htmem1 = g_new(MemoryRegion, 1);
	memory_region_init_alias(htmem1, NULL, "lowpcimem1", address_space_mem, 0x40000000, 0x40000000ULL);
	memory_region_add_subregion(address_space_mem, 0xe0010000000, htmem);
	memory_region_add_subregion(address_space_mem, 0xe0040000000, htmem1);

		/* allocate RAM */
	memory_region_init_ram(ram, NULL, "mips_r4k.ram", ram_size, &error_fatal);

	MemoryRegion *ram1 = g_new(MemoryRegion, 1);
	memory_region_init_alias(ram1, NULL, "lowmem", ram, 0, MIN(ram_size, 0x10000000));
	memory_region_add_subregion(address_space_mem, 0, ram1);
	memory_region_add_subregion(address_space_mem, 0x80000000ULL, ram);
	MemoryRegion *ram_pciram = g_new(MemoryRegion, 1);
	MemoryRegion *ram_pciram1 = g_new(MemoryRegion, 1);
	memory_region_init_alias(ram_pciram1, NULL, "ddrlowmem", ram, 0, 0x10000000);
	memory_region_init_alias(ram_pciram, NULL, "ddrmem", ram, 0, ram_size);


        memory_region_init(iomem_root, NULL,  "ls3a7a axi", UINT64_MAX);
	address_space_init(as,iomem_root, "ls3a7a axi memory");

	MemoryRegion *ram2 = g_new(MemoryRegion, 1);
	memory_region_init_alias(ram2, NULL, "dmalowmem", ram, 0, MIN(ram_size, 0x10000000));
        memory_region_add_subregion(iomem_root, 0, ram2);

	MemoryRegion *ram3 = g_new(MemoryRegion, 1);
	memory_region_init_alias(ram3, NULL, "dmamem", ram, 0, ram_size);
        memory_region_add_subregion(iomem_root, 0x80000000ULL, ram3);


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
    } else if ((flash_dinfo = drive_get_next(IF_PFLASH)))
	;
    else {
        bios = g_new(MemoryRegion, 1);
        memory_region_init_ram(bios, NULL, "mips_r4k.bios", BIOS_SIZE, &error_fatal);
        memory_region_set_readonly(bios, true);
        memory_region_add_subregion(get_system_memory(), 0x1fc00000, bios);

	bios_size = sizeof(aui_boot_code);
	rom_add_blob_fixed("bios",aui_boot_code,bios_size,0x1fc00000);
    }
    if (filename) {
        g_free(filename);
    }

    if (kernel_filename) {
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        reset_info[0]->vector = load_kernel(machine->dtb)?:reset_info[0]->vector;
    }


	ht_irq = ls3a_intctl_init(iomem_root, mycpu);

	/* Register 64 KB of IO space at 0x1f000000 */
	//isa_mmio_init(0x1ff00000, 0x00010000);
	//isa_mem_base = 0x10000000;
	cpu_irqset_bitmap = bitmap_new(smp_cpus*8);
	cpu_irqclr_bitmap = bitmap_new(smp_cpus*8);
	ls7a	=ls7a_intctl_init(address_space_mem, 0x10000000ULL, ht_irq[0]);
	ls3a7a_irq = ls7a->irqs;

	ls3auartpci_irq = qemu_allocate_irqs(ls3auartpci_set_irq, env, 3);
	ls7auart_irq = qemu_allocate_irqs(ls7auart_set_irq, ls3a7a_irq[8], 4);

    	pci_bus = bonito_init(&ls3auartpci_irq[2]);
	i = -1;
	address_space_write(&address_space_memory, 0x1fe0012c, MEMTXATTRS_UNSPECIFIED, (const uint8_t*)&i, 4);
	{

	PCIDevice *pci_dev = pci_create_multifunction(pci_bus,  PCI_DEVFN(2, 0), false, "rtl8139");
	DeviceState *dev = DEVICE(pci_dev);
	if(nd_table[3].used)
		qdev_set_nic_properties(dev, &nd_table[3]);
	DeviceClass *dc = DEVICE_GET_CLASS(dev);
	PCIDeviceClass *k = PCI_DEVICE_CLASS(DEVICE_CLASS(dc));
	k->romfile = NULL;
	dc->vmsd = NULL;
	qdev_init_nofail(dev);
	}

	if (serial_hd(0))
		serial_mm_init(address_space_mem, 0x1fe001e0, 0,ls3auartpci_irq[0],115200,serial_hd(0), DEVICE_NATIVE_ENDIAN);

	if (serial_hd(1))
		serial_mm_init(address_space_mem, 0x1fe001e8, 0,ls3auartpci_irq[1] ,115200,serial_hd(1), DEVICE_NATIVE_ENDIAN);

	if (serial_hd(2))
		serial_mm_init(address_space_mem, 0x10080000, 0,ls7auart_irq[0],115200,serial_hd(2), DEVICE_NATIVE_ENDIAN);

	if (serial_hd(3))
		serial_mm_init(address_space_mem, 0x10080100, 0,ls7auart_irq[1],115200,serial_hd(3), DEVICE_NATIVE_ENDIAN);

	if (serial_hd(4))
		serial_mm_init(address_space_mem, 0x10080200, 0,ls7auart_irq[2],115200,serial_hd(4), DEVICE_NATIVE_ENDIAN);

	if (serial_hd(5))
		serial_mm_init(address_space_mem, 0x10080300, 0,ls7auart_irq[3],115200,serial_hd(5), DEVICE_NATIVE_ENDIAN);

	{
		DeviceState *dev,*dev1;
		void *bus;
		qemu_irq cs_line;
		dev=sysbus_create_simple("ls1a_spi",0x1fe00220, NULL);
		bus = qdev_get_child_bus(dev, "ssi");
		if(flash_dinfo)
		{
			dev1 = ssi_create_slave_no_init(bus, "spi-flash");
			qdev_prop_set_drive(dev1, "drive", blk_by_legacy_dinfo(flash_dinfo), &error_fatal);
			qdev_prop_set_uint32(dev1, "size", 0x1000000);
			qdev_prop_set_uint64(dev1, "addr", 0x1d000000);
			qdev_init_nofail(dev1);
			ALIAS_REGION(0x1d000000ULL, 0x100000UL, 0x1fc00000ULL);
		}
		else dev1 = ssi_create_slave(bus, "ssi-sd");
		cs_line = qdev_get_gpio_in_named(dev1, "ssi-gpio-cs",  0);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1 , cs_line);
	}


	ppci_bus = pcibus_ls3a7a_init(0, ls3a7a_irq,pci_ls3a7a_map_irq, ram_pciram, ram_pciram1);



        sysbus_create_simple("ls2h_acpi", 0x100d0000, NULL);

	if (!getenv("E6882"))
	{
		PCIDevice *pci_dev = pci_create_multifunction(ppci_bus[1], -1, false, "e1000e");
		DeviceState *dev = DEVICE(pci_dev);
		if(nd_table[2].used)
			qdev_set_nic_properties(dev, &nd_table[2]);
		DeviceClass *dc = DEVICE_GET_CLASS(dev);
		PCIDeviceClass *k = PCI_DEVICE_CLASS(DEVICE_CLASS(dc));
		k->romfile = NULL;
		dc->vmsd = NULL;
		qdev_init_nofail(dev);
	} else {
		PCIDevice *d = pci_create_multifunction(ppci_bus[1], -1, true, "pciram");
		qdev_prop_set_uint32(DEVICE(d), "bar0", ~(0x08000000-1)|0x4);
		qdev_init_nofail(DEVICE(d));
		pci_set_word(d->config + PCI_CLASS_DEVICE, 0x0380);
		pci_set_word(d->config + PCI_VENDOR_ID, 0x1002);
		pci_set_word(d->config + PCI_DEVICE_ID, 0x6822);
	}

	pci_create_simple(ppci_bus[2], -1, "nec-usb-xhci");

#if 0
	{
		sysbus_create_simple("ls3a7a_wdt", 0x1fe7c060, NULL);
	}
#endif




#if 1
	{
		DeviceState *dev;
		SysBusDevice *s;
		dev = qdev_create(NULL, "ls1a_rtc");
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x100d0120);
		sysbus_connect_irq(s, 0, ls3a7a_irq[44]);
		sysbus_connect_irq(s, 1, ls3a7a_irq[45]);
		sysbus_connect_irq(s, 2, ls3a7a_irq[46]);
	}
#endif

	{
		DeviceState *dev;
		void *bus;
		static const uint8_t eeprom_spd[] = {
			0x92, 0x10, 0x0b, 0x02, 0x04, 0x21, 0x00, 0x01, 0x03, 0x11, 0x01, 0x08, 0x0a, 0x00, 0xfc, 0x00,
			0x6e, 0x78, 0x6e, 0x30, 0x6e, 0x11, 0x18, 0x86, 0x20, 0x08, 0x3c, 0x3c, 0x00, 0xf0, 0x83, 0x85,
			0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x11, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd1, 0xa2,
			0x48, 0x58, 0x4d, 0x53, 0x48, 0x34, 0x47, 0x55, 0x30, 0x33, 0x41, 0x31, 0x46, 0x31, 0x43, 0x2d,
			0x31, 0x36, 0x00, 0x00, 0x08, 0x1a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		};

		dev=sysbus_create_simple("ls1a_i2c",0x10090000, ls3a7a_irq[9]);
		bus = qdev_get_child_bus(dev, "i2c");
		i2c_create_slave(bus, "ds1338", 0x68);
		smbus_eeprom_init_one(bus, 0x50, eeprom_spd);
	}

	{
		DeviceState *dev;
		void *bus;
		dev=sysbus_create_simple("ls1a_i2c",0x10090100, ls3a7a_irq[9]);
		bus = qdev_get_child_bus(dev, "i2c");
		i2c_create_slave(bus, "ds1338", 0x68);
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
            sysbus_mmio_map(SYS_BUS_DEVICE(hpet), 0, 0x10001000);

            sysbus_connect_irq(SYS_BUS_DEVICE(hpet), 0, ls3a7a_irq[55]);
        }
	}


	{
                ddrcfg_iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(ddrcfg_iomem, NULL, &mips_qemu_ops, (void *)0x0ff00000, "ddr", 0x100000);
	}

	{

                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fe00180, "0x1fe00180", 0x8);
                memory_region_add_subregion(address_space_mem, 0x1fe00180, iomem);
		mips_qemu_writel((void *)0x1fe00180, 0, 0xff003390, 4);
	}

	{
		MemoryRegion *iomem;
                iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x3ff00200, "cachelock", 0x100);
                memory_region_add_subregion(address_space_mem, 0x3ff00200, iomem);
	}

	{
	#define ADDR 0xe000f000000
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
		memory_region_init_alias(iomem, NULL, "lowmem", ram, 0x0f000000, 0x10000);
		//memory_region_init_ram_nomigrate(iomem, NULL, "7aram", 0x10000, NULL);
                memory_region_add_subregion(address_space_mem, ADDR, iomem);
	#undef ADDR
 
	}




	SIMPLE_OPS(0xefdfb000178, 4);

	SIMPLE_OPS(0xefdfb000044, 4);

	SIMPLE_OPS(0xefdfe0001f4, 4);
	SIMPLE_OPS(0xefdfe000044, 4);

	SIMPLE_OPS(0x10013ff8, 4);

	SIMPLE_OPS(0x10010480, 0x50);

	SIMPLE_OPS(0x10010594, 4);
	SIMPLE_OPS(0x10010694, 4);
	SIMPLE_OPS(0x10010794, 4);
	SIMPLE_OPS(0x10010894, 4);

	SIMPLE_OPS(0x100105b4, 4);
	SIMPLE_OPS(0x100106b4, 4);
	SIMPLE_OPS(0x100107b4, 4);
	SIMPLE_OPS(0x100108b4, 4);

	SIMPLE_OPS(0x10010424, 4);
	iomem160 = SIMPLE_OPS(0x40000000, 0x1000);

	SIMPLE_OPS(0x100105d0,0x10);
	SIMPLE_OPS(0x100106d0,0x10);
	SIMPLE_OPS(0x100107d0,0x10);
	SIMPLE_OPS(0x100108d0,0x10);

	SIMPLE_OPS(0x100105f0,0x10);
	SIMPLE_OPS(0x100106f0,0x10);
	SIMPLE_OPS(0x100107f0,0x10);
	SIMPLE_OPS(0x100108f0,0x10);

	SIMPLE_OPS(0x10010610,0x10);
	SIMPLE_OPS(0x10010710,0x10);
	SIMPLE_OPS(0x10010810,0x10);
	SIMPLE_OPS(0x10010910,0x10);
	SIMPLE_OPS(0x1fe00100,0x4);
	SIMPLE_OPS(0xcfdfb000000,0x1000);
	SIMPLE_OPS(0x1fe001b0, 8);
	SIMPLE_OPS(0x100d0008, 4);
	SIMPLE_OPS(0x1fe001c0, 4);


	//mips_qemu_writel((void *)0xe0040000160, 0, 1<<24, 4);

	mypc_callback =  mypc_callback_for_net;
}


static void mips_machine_init(MachineClass *mc)
{
    mc->desc = "mips ls3a7a platform";
    mc->init = mips_ls3a7a_init;
    mc->max_cpus = 4;
    mc->block_default_type = IF_IDE;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("godson3");
}

DEFINE_MACHINE("ls3a7a", mips_machine_init)

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

#include "ls7a_int.c"
//-------------------------
// pci bridge
//-----------------

static int pci_ls3a7a_map_irq(PCIDevice *d, int pin)
{
	int dev=(d->devfn>>3)&0x1f;
	int fn=d->devfn& 7;
	int irq;

	if(pci_get_bus(d) != ls3a7a_pci_bus)
		return pin;

	switch(dev)
	{
		default:
		case 2:
		/*APB 2*/
		irq = 0;
		break;

		case 3:
		/*GMAC0 3 0*/
		/*GMAC1 3 1*/
		irq = (fn == 0) ? 12 : 14;
		break;

		case 4:
		/* ohci:4 0 */
		/* ehci:4 1 */
		irq = (fn == 0) ? 49 : 48;
		break;

		case 5:
		/* ohci:5 0 */
		/* ehci:5 1 */
		irq = (fn == 0) ? 51 : 50;
		break;

		case 6:
		/* DC: 6 1 28 */
		/* GPU:6 0 29 */
		irq = (fn == 0) ? 29 : 28;
		break;

		case 7:
		/*HDA: 7 0 58 */
		irq = 58;
		break;

		case 8:
		/* sata */
		if (fn == 0)
			irq = 16;
		if (fn == 1)
			irq = 17;
		if (fn == 2)
			irq = 18;
		break;

		case 9:
		/* pcie_f0 port0 */
		irq = 32;
		break;

		case 10:
		/* pcie_f0 port1 */
		irq = 33;
		break;

		case 11:
		/* pcie_f0 port2 */
		irq = 34;
		break;

		case 12:
		/* pcie_f0 port3 */
		irq = 35;
		break;

		case 13:
		/* pcie_f1 port0 */
		irq = 36;
		break;

		case 14:
		/* pcie_f1 port1 */
		irq = 37;
		break;

		case 15:
		/* pcie_g0 port0 */
		irq = 40;
		break;

		case 16:
		/* pcie_g0 port1 */
		irq = 41;
		break;

		case 17:
		/* pcie_g1 port0 */
		irq = 42;
		break;

		case 18:
		/* pcie_g1 port1 */
		irq = 43;
		break;

		case 19:
		/* pcie_h port0 */
		irq = 38;
		break;

		case 20:
		/* pcie_h port1 */
		irq = 39;
		break;
	}
	return irq;
}

static void pci_ls3a7a_set_irq(void *opaque, int irq_num, int level)
{
		qemu_set_irq(ls3a7a_irq[irq_num],level);
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


#define TYPE_BONITO_PCI_HOST_BRIDGE "ls3a7a-pcihost"
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
	MemoryRegion ls7aextcfg_mem;
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
    MemoryRegion iomem_submem;
    MemoryRegion iomem_subbigmem;
    MemoryRegion iomem_io;
    AddressSpace as_mem;
    AddressSpace as_io;
   int (*pci_map_irq)(PCIDevice *d, int irq_num);
};

static void pci_ls3a7a_config_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	PCIBonitoState *s = opaque;
	//   PCIDevice *d = PCI_DEVICE(s);
	BonitoState *phb = s->pcihost;

	if(!(addr&0x1000000)) addr &= 0xffffff;

	pci_data_write(phb->bus,  addr, val, size);
}

static uint64_t pci_ls3a7a_config_readl (void *opaque, hwaddr addr, unsigned size)
{
	PCIBonitoState *s = opaque;
	//  PCIDevice *d = PCI_DEVICE(s);
	BonitoState *phb = s->pcihost;
	uint32_t val;

	if(!(addr&0x1000000)) addr &= 0xffffff;


	val = pci_data_read(phb->bus, addr, size);
	//printf("pci_ls3a7a_config_readl 0x%x 0x%x\n", (int)addr, val);
	return val;
}


static const MemoryRegionOps pci_ls3a7a_config_ops = {
    .read = pci_ls3a7a_config_readl,
    .write = pci_ls3a7a_config_writel,
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

static void bonito_initfn(PCIDevice *dev, Error **errp)
{
    PCIBonitoState *s = DO_UPCAST(PCIBonitoState, parent_obj.parent_obj, dev);
    SysBusDevice *sysbus = SYS_BUS_DEVICE(s->pcihost);

    pci_bridge_initfn(dev, TYPE_PCI_BUS);

    /* set the north bridge pci configure  mapping */
    memory_region_init_io(&s->conf_mem, NULL, &bonito_pciconf_ops, s,
                          "north-bridge-pci-config", 0x100);
    sysbus_init_mmio(sysbus, &s->conf_mem);

    /* set the south bridge pci configure  mapping */
    memory_region_init_io(&s->data_mem, NULL, &pci_ls3a7a_config_ops, s,
                          "south-bridge-pci-config", 0x2000000);
    sysbus_init_mmio(sysbus, &s->data_mem);

    /* set the south bridge pci configure  mapping */
    memory_region_init_io(&s->ls7aextcfg_mem, NULL, &pci_ls3a7a_config_ops, s,
                          "south-bridge-pci-config", 0x20000000);
    sysbus_init_mmio(sysbus, &s->ls7aextcfg_mem);

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
    pci_set_word(dev->config + PCI_CLASS_DEVICE, 0x0604);

}


static void bonito_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = bonito_initfn;
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
    .name          = "LS7A_Bonito",
    .parent        = TYPE_PCI_BRIDGE,
    .instance_size = sizeof(PCIBonitoState),
    .class_init    = bonito_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static AddressSpace *pci_dma_context_fn(PCIBus *bus, void *opaque, int devfn);

#define MAX_SATA_PORTS     6
static PCIBus **pcibus_ls3a7a_init(int busno, qemu_irq *pic, int (*board_map_irq)(PCIDevice *d, int irq_num), MemoryRegion *ram, MemoryRegion *ram1)
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

    memory_region_add_subregion(&pcihost->iomem_mem, 0x0UL, ram1);
    memory_region_add_subregion(&pcihost->iomem_mem, 0x80000000ULL, ram);

    /* set the pcihost pointer before bonito_initfn is called */
    //d = pci_create(phb->bus, PCI_DEVFN(0, 0), "LS7A_Bonito");
    for (i=0;i<4;i++)
    {
    d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(9+i, 0), true, "LS7A_Bonito");

    s = DO_UPCAST(PCIBonitoState, parent_obj.parent_obj, d);
    s->pcihost = pcihost;
    pcihost->pci_dev = s;
    br = PCI_BRIDGE(d);
    pci_bridge_map_irq(br, NULL, board_map_irq);
    qdev_init_nofail(DEVICE(d));
    bus2 = pci_bridge_get_sec_bus(br);

    pci_setup_iommu(bus2, pci_dma_context_fn, pcihost);
    pci_bus[i] = bus2;
    }

    d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(3, 0), true, "pci-synopgmac");
    dev = DEVICE(d);
    if(nd_table[0].used)
    qdev_set_nic_properties(dev, &nd_table[0]);
    qdev_prop_set_int32(dev, "enh_desc", 1);
    qdev_init_nofail(DEVICE(d));
    pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
    pci_set_word(d->config + PCI_DEVICE_ID, 0x7a03);

    d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(3, 1), true, "pci-synopgmac");
    dev = DEVICE(d);
    if(nd_table[1].used)
    qdev_set_nic_properties(dev, &nd_table[1]);
    qdev_prop_set_int32(dev, "enh_desc", 1);
    qdev_init_nofail(DEVICE(d));
    pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
    pci_set_word(d->config + PCI_DEVICE_ID, 0x7a03);

    d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(4, 0), true, "pci-ohci");
    qdev_init_nofail(DEVICE(d));
    pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
    pci_set_word(d->config + PCI_DEVICE_ID, 0x7a24);

#if 1
    d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(4, 1), true, "usb-ehci");
    qdev_init_nofail(DEVICE(d));
    pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
    pci_set_word(d->config + PCI_DEVICE_ID, 0x7a14);
#endif

    /* ahci and SATA device, for q35 1 ahci controller is built-in */
    d= pci_create_simple_multifunction(pcihost->bus,
                                           PCI_DEVFN(8,0),
                                           true, "ls2k-ahci");

    pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
    pci_set_word(d->config + PCI_DEVICE_ID, 0x7a08);

    ide_drive_get(hd, LS2K_AHCI(d)->ahci.ports);
    ls2k_ahci_ide_create_devs(d, hd);

    {
    //confbus
	    MemoryRegion *iomem = g_new(MemoryRegion, 1);
	    memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)CONFBASE, "confbus", 0x8000);
	printf("iomem=%p\n", iomem);
	    d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(21, 0), true, "pciram");
	    qdev_prop_set_uint32(DEVICE(d), "bar0", ~(0x00008000-1)|4);
	    qdev_prop_set_ptr(DEVICE(d), "iomem0", iomem);
	    qdev_init_nofail(DEVICE(d));
    pci_set_word(d->wmask + PCI_COMMAND, 0);
    }

    {
	    //gpu
	    MemoryRegion *iomem = g_new(MemoryRegion, 1);
	    memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)GPUBASE, "gpu", 0x8000);
	    d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(6, 0), true, "pciram");
	    qdev_prop_set_uint32(DEVICE(d), "bar0", ~(0x00008000-1)|4);
	    qdev_prop_set_ptr(DEVICE(d), "iomem0", iomem);
	    qdev_prop_set_uint32(DEVICE(d), "bar2", ~(0x08000000-1)|0x4);
	    qdev_init_nofail(DEVICE(d));
	    pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
	    pci_set_word(d->config + PCI_DEVICE_ID, 0x7a15);
    	    pci_set_word(d->config + PCI_CLASS_DEVICE, PCI_CLASS_MULTIMEDIA_VIDEO);
    }

#if 0
    {
	    //lpc
	    MemoryRegion *iomem = g_new(MemoryRegion, 1);
	    memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)LPCBASE, "lpc", 0x8000);
	    d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(6, 0), true, "pciram");
	    qdev_prop_set_uint32(DEVICE(d), "bar0", ~(0x00001000-1)|4);
	    qdev_prop_set_ptr(DEVICE(d), "iomem0", iomem);
	    qdev_prop_set_uint32(DEVICE(d), "bar1", ~(0x08000000-1)|4);
	    qdev_prop_set_uint32(DEVICE(d), "bar2", ~(0x00000100-1)|4);
	    qdev_init_nofail(DEVICE(d));
	    pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
	    pci_set_word(d->config + PCI_DEVICE_ID, 0x7a15);
    }
#endif

    {
            //dc
            void *bus;
            DeviceState *i2c_dev;
            static uint8_t edid[] = {
                    0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x5a, 0x63,
                    0x22, 0x1a, 0x01, 0x01, 0x01, 0x01,
                    0x05, 0x14, 0x01, 0x03, 0x08, 0x29, 0x17, 0x78, 0x2e, 0x3d,
                    0x85, 0xa6, 0x56, 0x4a, 0x9a, 0x24,
                    0x12, 0x50, 0x54, 0xbf, 0xef, 0x80, 0x81, 0x80, 0x81, 0x40,
                    0x71, 0x4f, 0x01, 0x01, 0x01, 0x01,
                    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x66, 0x21, 0x56, 0xaa,
                    0x51, 0x00, 0x1e, 0x30, 0x46, 0x8f,
                    0x33, 0x00, 0x9a, 0xe6, 0x10, 0x00, 0x00, 0x1e, 0x00, 0x00,
                    0x00, 0xff, 0x00, 0x52, 0x39, 0x4a,
                    0x31, 0x30, 0x30, 0x35, 0x32, 0x37, 0x36, 0x37, 0x39, 0x0a,
                    0x00, 0x00, 0x00, 0xfd, 0x00, 0x32,
                    0x4b, 0x18, 0x52, 0x0e, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20,
                    0x20, 0x20, 0x00, 0x00, 0x00, 0xfc,
                    0x00, 0x56, 0x41, 0x31, 0x39, 0x31, 0x33, 0x77, 0x0a, 0x20,
                    0x20, 0x20, 0x20, 0x20, 0x00, 0xfb,
            };
            d = pci_create_simple_multifunction(pcihost->bus, PCI_DEVFN(6, 1), true,
                                                "pci_ls2h_fb");
            pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
            pci_set_word(d->config + PCI_DEVICE_ID, 0x7a06);
            pci_set_word(d->config + PCI_CLASS_DEVICE, PCI_CLASS_DISPLAY_VGA);

            i2c_dev = sysbus_create_simple("gpio_i2c", -1, NULL);
            /* I2C read data */
            qdev_connect_gpio_out(i2c_dev, 0, qdev_get_gpio_in(d, 0));
            /* I2C data */
            qdev_connect_gpio_out(d, 0, qdev_get_gpio_in(i2c_dev, 0));
            /* I2C clock */
            qdev_connect_gpio_out(d, 1, qdev_get_gpio_in(i2c_dev, 1));
            bus = qdev_get_child_bus(i2c_dev, "i2c");
            smbus_eeprom_init_one(bus, 0x50, edid);
    }

    d = pci_create_simple_multifunction(pcihost->bus, PCI_DEVFN(22,0), true, "pci-1a_spi");
    {
	    DeviceState *dev1;
	    void *bus;
	    qemu_irq cs_line;
	    DriveInfo *flash_dinfo=drive_get_next(IF_PFLASH);
	    if(flash_dinfo)
	    {
		    bus = qdev_get_child_bus(DEVICE(d), "ssi");
		    dev1 = ssi_create_slave_no_init(bus, "spi-flash");
		    qdev_prop_set_drive(dev1, "drive", blk_by_legacy_dinfo(flash_dinfo), &error_fatal);
		    qdev_init_nofail(dev1);
		    cs_line = qdev_get_gpio_in_named(dev1, "ssi-gpio-cs",  0);
		    qdev_connect_gpio_out_named(DEVICE(d), SYSBUS_DEVICE_GPIO_IRQ, 0, cs_line);
	    }
	    //else dev1 = ssi_create_slave(bus, "ssi-sd");
    }


   {
    BusState *hdabus;
    DeviceState *codec;
    d = pci_create_simple_multifunction(pcihost->bus, PCI_DEVFN(7,0), true, "intel-hda");
    pci_set_word(d->config + PCI_VENDOR_ID, 0x0014);
    pci_set_word(d->config + PCI_DEVICE_ID, 0x7a07);

    hdabus = QLIST_FIRST(&DEVICE(d)->child_bus);
    codec = qdev_create(hdabus, "hda-duplex");
    qdev_init_nofail(codec);
   }

    sysbus = SYS_BUS_DEVICE(s->pcihost);
     /*self header*/
    sysbus_mmio_map(sysbus, 0, 0xfe00004800);
     /*devices header*/
    sysbus_mmio_map(sysbus, 1, 0x1a000000);
    ALIAS_REGION(0x1a000000ULL, 0x02000000ULL, 0xefdfe000000);
    sysbus_mmio_map(sysbus, 2, 0xefe00000000);

    memory_region_add_subregion(get_system_memory(), 0x10000000UL, &pcihost->iomem_submem);
    memory_region_add_subregion(get_system_memory(), 0x40000000UL, &pcihost->iomem_subbigmem);
    memory_region_add_subregion(get_system_memory(), 0x18000000UL, &pcihost->iomem_io);
    ALIAS_REGION(0x18000000ULL, 0x10000000ULL, 0xefdfc000000UL);
    ALIAS_REGION(0x10000000ULL, 0x10000000ULL, 0xe0010000000UL);

//    MemoryRegion *iomem_subbigmem1 = g_new(MemoryRegion, 1);
//    memory_region_init_alias(iomem_subbigmem1, NULL, "pcisubmem1", &pcihost->iomem_mem, 0x40000000, 0x40000000);
//    memory_region_add_subregion(get_system_memory(), 0xe0040000000UL, iomem_subbigmem1);

//pci-synopgmac


    ls3a7a_pci_bus = pcihost->bus;

    return pci_bus;
}


static AddressSpace *pci_dma_context_fn(PCIBus *bus, void *opaque, int devfn)
{
    BonitoState *pcihost = opaque;
    return &pcihost->as_mem;
}


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
    pcihost = BONITO_PCI_HOST_BRIDGE(dev);

    memory_region_init(&pcihost->iomem_mem, OBJECT(pcihost), "system", INT64_MAX);
    address_space_init(&pcihost->as_mem, &pcihost->iomem_mem, "pcie memory");

    /* Host memory as seen from the PCI side, via the IOMMU.  */

    memory_region_init_alias(&pcihost->iomem_submem, NULL, "pcisubmem", &pcihost->iomem_mem, 0x10000000, 0x2000000);
    memory_region_init_alias(&pcihost->iomem_subbigmem, NULL, "pcisubmem", &pcihost->iomem_mem, 0x40000000, 0x40000000);

    memory_region_init(&pcihost->iomem_io, OBJECT(pcihost), "system", 0x10000);
    address_space_init(&pcihost->as_io, &pcihost->iomem_io, "pcie io");

    phb = PCI_HOST_BRIDGE(dev);
    pcihost->bus = phb->bus = pci_register_root_bus(DEVICE(dev), NULL,
                                pci_ls3a7a_set_irq, pcihost->pci_map_irq, pcihost->pic,
                                &pcihost->iomem_mem, &pcihost->iomem_io,
                                PCI_DEVFN(0, 0), 64, TYPE_PCIE_BUS);


    pci_setup_iommu(pcihost->bus, pci_dma_context_fn, pcihost);

}

static const char *ls3a7a_host_root_bus_path(PCIHostState *host_bridge,
                                          PCIBus *rootbus)
{
    return "0000:00";
}

static void bonito_pcihost_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIHostBridgeClass *hc = PCI_HOST_BRIDGE_CLASS(klass);

    hc->root_bus_path = ls3a7a_host_root_bus_path;
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
#define LS7A
#define LOONGSON_3ASINGLE
#include "loongson_bootparam.c"
//-------------------------

#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "hw/pci/pci_host.h"

#undef DPRINTF
//#define DEBUG_IRQ

#ifdef DEBUG_IRQ
#define DPRINTF(fmt, args...) \
do { printf("IRQ: " fmt , ##args); } while (0)
#else
#define DPRINTF(fmt, args...)
#endif


#define MAX_PILS 16


#define HT_CONTROL_REGS_BASE   0xefdfb000000LL
#define HT1LO_PCICFG_BASE      0xefdfe000000LL
#define HT1LO_PCICFG_BASE_TP1  0xefdff000000LL
#define HT1LO_PCICFG_BASE_ALIAS      0x1a000000
#define INT_ROUTER_REGS_BASE   0x3ff01400


#define INT_ROUTER_REGS_SYS_INT0	0x00
#define INT_ROUTER_REGS_SYS_INT1	0x01
#define INT_ROUTER_REGS_SYS_INT2	0x02
#define INT_ROUTER_REGS_SYS_INT3	0x03
#define INT_ROUTER_REGS_PCI_INT0	0x04
#define INT_ROUTER_REGS_PCI_INT1	0x05
#define INT_ROUTER_REGS_PCI_INT2	0x06
#define INT_ROUTER_REGS_PCI_INT3	0x07
#define INT_ROUTER_REGS_MATRIX_INT0	0x08
#define INT_ROUTER_REGS_MATRIX_INT1	0x09
#define INT_ROUTER_REGS_LPC_INT		0x0a
#define INT_ROUTER_REGS_MC0		0x0B
#define INT_ROUTER_REGS_MC1		0x0C
#define INT_ROUTER_REGS_BARRIER		0x0d
#define INT_ROUTER_REGS_RESERVE		0x0e
#define INT_ROUTER_REGS_PCI_PERR	0x0f

#define INT_ROUTER_REGS_HT0_INT0	0x10
#define INT_ROUTER_REGS_HT0_INT1	0x11
#define INT_ROUTER_REGS_HT0_INT2	0x12
#define INT_ROUTER_REGS_HT0_INT3	0x13
#define INT_ROUTER_REGS_HT0_INT4	0x14
#define INT_ROUTER_REGS_HT0_INT5	0x15
#define INT_ROUTER_REGS_HT0_INT6	0x16
#define INT_ROUTER_REGS_HT0_INT7	0x17
#define INT_ROUTER_REGS_HT1_INT0	0x18
#define INT_ROUTER_REGS_HT1_INT1	0x19
#define INT_ROUTER_REGS_HT1_INT2	0x1a
#define INT_ROUTER_REGS_HT1_INT3	0x1b
#define INT_ROUTER_REGS_HT1_INT4	0x1c
#define INT_ROUTER_REGS_HT1_INT5	0x1d
#define INT_ROUTER_REGS_HT1_INT6	0x1e
#define INT_ROUTER_REGS_HT1_INT7	0x1f
#define IO_CONTROL_REGS_INTISR  	0x20
#define IO_CONTROL_REGS_INTEN		0x24	
#define IO_CONTROL_REGS_INTENSET	0x28	
#define IO_CONTROL_REGS_INTENCLR	0x2c	
#define IO_CONTROL_REGS_INTEDGE		0x38	
#define IO_CONTROL_REGS_CORE0_INTISR	0x40	
#define IO_CONTROL_REGS_CORE1_INTISR	0x48	
#define IO_CONTROL_REGS_CORE2_INTISR	0x50	
#define IO_CONTROL_REGS_CORE3_INTISR	0x58	





#define HT_LINK_CONFIG_REG  0x44
#define HT_ROUTE_MODE		0x58
#define HT_IRQ_VECTOR_REG0	0x80	
#define HT_IRQ_VECTOR_REG1	0x84	
#define HT_IRQ_VECTOR_REG2	0x88	
#define HT_IRQ_VECTOR_REG3	0x8C	
#define HT_IRQ_VECTOR_REG4	0x90	
#define HT_IRQ_VECTOR_REG5	0x94	
#define HT_IRQ_VECTOR_REG6	0x98	
#define HT_IRQ_VECTOR_REG7	0x9C	

#define HT_IRQ_ENABLE_REG0	0xA0	
#define HT_IRQ_ENABLE_REG1	0xA4	
#define HT_IRQ_ENABLE_REG2	0xA8	
#define HT_IRQ_ENABLE_REG3	0xAC	
#define HT_IRQ_ENABLE_REG4	0xB0	
#define HT_IRQ_ENABLE_REG5	0xB4	
#define HT_IRQ_ENABLE_REG6	0xB8	
#define HT_IRQ_ENABLE_REG7	0xBC	

#define HT_UNCACHE_ENABLE_REG0	0xF0
#define HT_UNCACHE_BASE_REG0	0xF4
#define HT_UNCACHE_ENABLE_REG1	0xF8
#define HT_UNCACHE_BASE_REG1	0xFC



typedef struct LS3a_INTCTLState {
	unsigned char int_route_reg[0x100];
	unsigned char ht_irq_reg[0x100];
 	CPUMIPSState **env;
#ifdef DEBUG_IRQ_COUNT
    uint64_t irq_count[32];
#endif
    uint32_t pil_out[MAX_CPUS];
} LS3a_INTCTLState;

typedef struct LS3a_func_args {
 LS3a_INTCTLState *state;
 uint64_t base;
 uint32_t mask;
 uint8_t *mem;
} LS3a_func_args;
int ht_route_mode;


static void ht_update_irq(void *opaque,int disable);

// per-cpu interrupt controller
static uint32_t ls3a_intctl_mem_readb(void *opaque, hwaddr addr)
{
	LS3a_func_args *a = opaque;
    uint32_t ret;

	addr &= a->mask;

	ret=*(uint8_t *)(a->mem+addr);

    DPRINTF("read reg 0x" TARGET_FMT_plx " = %x\n", addr + s->base, ret);

    return ret;
}

static uint32_t ls3a_intctl_mem_readw(void *opaque, hwaddr addr)
{
	LS3a_func_args *a = opaque;
    uint32_t ret;

	addr &= a->mask;

	ret=*(uint16_t *)(a->mem+addr);

    DPRINTF("read reg 0x" TARGET_FMT_plx " = %x\n", addr + s->base, ret);

    return ret;
}

static uint32_t ls3a_intctl_mem_readl(void *opaque, hwaddr addr)
{
	LS3a_func_args *a = opaque;
	uint32_t ret;
	static int linkcfg=0;
	LS3a_INTCTLState *s = a->state;

	addr &= a->mask;

	switch(a->base+addr)
	{
	case HT_CONTROL_REGS_BASE+HT_ROUTE_MODE	:
	 ret = ht_route_mode;
	break;
	case HT_CONTROL_REGS_BASE+HT_LINK_CONFIG_REG:
		ret = linkcfg;
		linkcfg =random();
		//printf("ret=%x\n",ret);
		break;
	case HT_CONTROL_REGS_BASE + HT_IRQ_VECTOR_REG2:
		ret = *(uint32_t *)(s->ht_irq_reg+HT_IRQ_VECTOR_REG2);
		//address_space_read(&address_space_memory, 0x100003a0, MEMTXATTRS_UNSPECIFIED, &ret, 4);
	break;
	case HT_CONTROL_REGS_BASE + HT_IRQ_VECTOR_REG3:
		ret = *(uint32_t *)(s->ht_irq_reg+HT_IRQ_VECTOR_REG3);
		//address_space_read(&address_space_memory, 0x100003a4, MEMTXATTRS_UNSPECIFIED, &ret, 4);
	break;
	case INT_ROUTER_REGS_BASE + IO_CONTROL_REGS_CORE0_INTISR:
	ret = 0x0f000000|((!!uart_irqstatus)<<10)|(pci_irqstatus << 4);
	break;
	default:
	ret=*(uint32_t *)(a->mem+addr);
	}

    DPRINTF("read reg 0x" TARGET_FMT_plx " = %x\n", addr + s->base, ret);

    return ret;
}

static void ls3a_intctl_mem_writeb(void *opaque, hwaddr addr, uint32_t val)
{
	LS3a_func_args *a = opaque;
	LS3a_INTCTLState *s = a->state;

	addr &= a->mask;

	switch(a->base+addr)
	{
	case INT_ROUTER_REGS_BASE+INT_ROUTER_REGS_HT1_INT0 ... INT_ROUTER_REGS_BASE+INT_ROUTER_REGS_HT1_INT7:
	{
		uint32_t old;
		old = *(uint8_t *)(a->mem + addr);
		if (old != val)	
		 ht_update_irq(s,1);
		*(uint8_t *)(a->mem + addr) = val;
		 ht_update_irq(s,0);
	}
	break;
	case HT_CONTROL_REGS_BASE+HT_IRQ_VECTOR_REG0 ...  HT_CONTROL_REGS_BASE+HT_IRQ_VECTOR_REG7:
	*(uint8_t *)(a->mem + addr) &= ~val;
	ht_update_irq(s,0);
	break;

	case HT_CONTROL_REGS_BASE+HT_IRQ_ENABLE_REG0 ... HT_CONTROL_REGS_BASE+HT_IRQ_ENABLE_REG7:
	*(uint8_t *)(a->mem + addr) = val;
	ht_update_irq(s,0);
	break;
		
	default:
	*(uint8_t *)(a->mem + addr) = val;
	break;
	}
}

static void ls3a_intctl_mem_writew(void *opaque, hwaddr addr, uint32_t val)
{
	LS3a_func_args *a = opaque;

	addr &= a->mask;
	*(uint16_t *)(a->mem + addr) = val;
}

static void ls3a_intctl_mem_writel(void *opaque, hwaddr addr, uint32_t val)
{
	LS3a_func_args *a = opaque;
	LS3a_INTCTLState *s = a->state;

	addr &= a->mask;
//	printf("base=%llx,addr=%llx,mask=%x,val=%08x\n",a->base,addr,a->mask,val);
	switch(a->base+addr)
	{
	case INT_ROUTER_REGS_BASE+INT_ROUTER_REGS_HT1_INT0:
	{
		uint32_t old;
		old = *(uint32_t *)(a->mem + addr);
		if (old != val)	
		 ht_update_irq(s,1);
		*(uint32_t *)(a->mem + addr) = val;
		 ht_update_irq(s,0);
		
	}
	break;
	case INT_ROUTER_REGS_BASE + IO_CONTROL_REGS_INTENSET:
	*(uint32_t *)(a->mem + IO_CONTROL_REGS_INTEN) |= val;
	break;
	case INT_ROUTER_REGS_BASE + IO_CONTROL_REGS_INTENCLR:
	*(uint32_t *)(a->mem + IO_CONTROL_REGS_INTEN) &= ~val;
	break;

	case HT_CONTROL_REGS_BASE+HT_IRQ_VECTOR_REG0 ... HT_CONTROL_REGS_BASE+HT_IRQ_VECTOR_REG7:
	*(uint32_t *)(a->mem + addr) &= ~val;
	ht_update_irq(s,0);
	break;

	case HT_CONTROL_REGS_BASE+HT_IRQ_ENABLE_REG0 ... HT_CONTROL_REGS_BASE+HT_IRQ_ENABLE_REG7:
	*(uint32_t *)(a->mem + addr) = val;
	ht_update_irq(s,0);
	break;

	case HT_CONTROL_REGS_BASE + HT_ROUTE_MODE:
	 ht_route_mode = val;
	 ht_update_irq(s,0);
	break;
		
	default:
	*(uint32_t *)(a->mem + addr) = val;
	break;
	}
}

static const MemoryRegionOps ls3a_intctl_ops = {
    .old_mmio = {
	    .read = {
		    ls3a_intctl_mem_readb,
		    ls3a_intctl_mem_readw,
		    ls3a_intctl_mem_readl,
	    },
	    .write = {
		    ls3a_intctl_mem_writeb,
		    ls3a_intctl_mem_writew,
		    ls3a_intctl_mem_writel,
	    }
	}
	,
		.endianness = DEVICE_NATIVE_ENDIAN,
};

static int ls_raise_cpuirq(LS3a_INTCTLState *s, int introute, uint64_t isr, uint64_t ier, int disable)
{
	uint32_t core,irq_nr;
	core = introute&0xf;
	irq_nr = ((introute>>4)&0xf);

	//printf("ht_update_irq%d core:%d irq_nr:%d disable %d\n", i, core, irq_nr, disable);

	if(core>0 && irq_nr>0)
	{
		if(isr&ier && !disable)
			set_bit((ffs(core)-1)*8 + ffs(irq_nr)+1, cpu_irqset_bitmap);
		else
			set_bit((ffs(core)-1)*8 + ffs(irq_nr)+1, cpu_irqclr_bitmap);
	}

    return 0;
}

static int ls_process_cpuirq(LS3a_INTCTLState *s)
{
	int bit;
	while (1) {
		bit = find_first_bit(cpu_irqset_bitmap, smp_cpus * 8);
		if (bit == smp_cpus * 8)
			break;
		qemu_irq_raise(s->env[bit/8]->irq[bit%8]);
		clear_bit(bit, cpu_irqset_bitmap);
		clear_bit(bit, cpu_irqclr_bitmap);
	}
	while (1) {
		bit = find_first_bit(cpu_irqclr_bitmap, smp_cpus * 8);
		if (bit == smp_cpus * 8)
			break;
		qemu_irq_lower(s->env[bit/8]->irq[bit%8]);
		clear_bit(bit, cpu_irqclr_bitmap);
	}

    return 0;
}

static void ht_update_irq(void *opaque,int disable)
{
	LS3a_INTCTLState *s = opaque;
	uint64_t isr,ier;
	uint32_t irtr;
	int i;
	isr = ls7a->intreg_pending & ~ls7a->int_mask;
	ier = (*(uint32_t *)(s->int_route_reg + IO_CONTROL_REGS_INTEN) & 1)?ls7a->route_int[0] :0;
	irtr = *(uint8_t *)(s->int_route_reg+INT_ROUTER_REGS_SYS_INT0);
	ls_raise_cpuirq(s, irtr, isr, ier, disable);

	ier = (*(uint32_t *)(s->int_route_reg + IO_CONTROL_REGS_INTEN) & 2)?ls7a->route_int[1] :0;
	irtr = *(uint8_t *)(s->int_route_reg+INT_ROUTER_REGS_SYS_INT1);
	ls_raise_cpuirq(s, irtr, isr, ier, disable);

	for (i = 0; i < 8; i++)
		ls7a->msiroute_ht[i] = 0;

	for(i=0;i<64;i++)
	{
		if (isr & ls7a->htmsi_en & (1ULL<<i)) {
		unsigned char t;
		t = ls7a->msiroute[i];
		ls7a->msiroute_ht[t/32] |= 1<<(t%32);
		}
	}

	//*(uint64_t *) s->ht_irq_reg + 0x58
	for (i =0;i < 8; i++) {
		*(uint32_t *)(s->ht_irq_reg+HT_IRQ_VECTOR_REG0 + i*4) = ls7a->msiroute_ht[i]; 
		ier = *(uint32_t *)(s->ht_irq_reg+HT_IRQ_ENABLE_REG0 + i *4);
	 if((ht_route_mode & 0x700) == 0x400)
		irtr = *(uint8_t *)(s->int_route_reg+INT_ROUTER_REGS_HT1_INT0 + i);
	 else
		irtr = *(uint8_t *)(s->int_route_reg+INT_ROUTER_REGS_HT1_INT0 + (i/2));
		isr = ls7a->msiroute_ht[i];

		ls_raise_cpuirq(s, irtr, isr, ier, disable);
	}

	ls_process_cpuirq(s);

}

static void ht_set_irq(void *opaque, int irq, int level)
{
	ht_update_irq(opaque,0);
}

static qemu_irq *ls3a_intctl_init(MemoryRegion *iomem_root, CPUMIPSState *env[])
{
	qemu_irq *ht_irq;
	LS3a_INTCTLState *s;
	LS3a_func_args *a_irqrouter,*a_htirq;

	s = g_malloc0(sizeof(LS3a_INTCTLState));
	if (!s)
		return NULL;

	a_irqrouter=g_malloc0(sizeof(LS3a_func_args));
	a_htirq=g_malloc0(sizeof(LS3a_func_args));
	a_irqrouter->state = s;
	a_irqrouter->base = INT_ROUTER_REGS_BASE;
	a_irqrouter->mem = s->int_route_reg;
	a_irqrouter->mask = 0xff;
	a_htirq->state = s;
	a_htirq->base = HT_CONTROL_REGS_BASE;
	a_htirq->mem = s->ht_irq_reg;
	a_htirq->mask = 0xff;

	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &ls3a_intctl_ops, a_irqrouter, "ls3a_intctl", 256);
                memory_region_add_subregion(get_system_memory(), INT_ROUTER_REGS_BASE, iomem);
	}

	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &ls3a_intctl_ops, a_htirq, "ls3a_intctl", 256);
                memory_region_add_subregion(get_system_memory(), HT_CONTROL_REGS_BASE, iomem);
	}


	s->env = env;

	ht_irq = qemu_allocate_irqs(ht_set_irq, s, 1);



    return ht_irq;
}
