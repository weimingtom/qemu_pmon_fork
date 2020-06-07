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
#include "hw/pci/pcie_host.h"
#include "loongson_bootparam.h"
#include <stdlib.h>

#include "loongson3a_rom.h"

#ifdef DEBUG_LS3A
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "i82374: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) \
do {} while (0)
#endif

#define PHYS_TO_VIRT(x) ((x) | ~(target_ulong)0x7fffffff)

#define VIRT_TO_PHYS_ADDEND (-((int64_t)(int32_t)0x80000000))

#define MAX_IDE_BUS 2
#define TARGET_REALPAGE_MASK (TARGET_PAGE_MASK<<2)

#define sysbus_mmio_map_iomem(iomem, busdev, n, devaddr) \
{ \
 SysBusDevice *s = busdev; \
 s->mmio[n].addr = devaddr; \
 memory_region_add_subregion(iomem, devaddr, s->mmio[n].memory); \
}

static const int ide_iobase[2] = { 0x1f0, 0x170 };
static const int ide_iobase2[2] = { 0x3f6, 0x376 };
static const int ide_irq[2] = { 14, 15 };

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
#define align(x) (((x)+15)&~15)
static int pci_ls3a2h_map_irq(PCIDevice *d, int irq_num);


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


static MemoryRegion *ddrcfg_iomem;
static int reg180;
extern target_ulong mypc;

static void mips_qemu_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case 0x1fe00180:
			reg180 = val;
			memory_region_transaction_begin();
			if(ddrcfg_iomem->container == get_system_memory())
				memory_region_del_subregion(get_system_memory(), ddrcfg_iomem);

			if((val&0x10) == 0)
			{
				memory_region_add_subregion_overlap(get_system_memory(), 0x0ff00000, ddrcfg_iomem, 1);
			}

			memory_region_transaction_commit();
			break;
		case 0x1fd00210:
		break;
		case 0x1fd00220:
		break;
	}
}

static uint64_t mips_qemu_readl (void *opaque, hwaddr addr, unsigned size)
{
	addr=((hwaddr)(long)opaque) + addr;
	switch(addr)
	{
		case 0x1fe00180:
		return reg180;
		case 0x1fd00210:
		return 0x049bbf00;
		case 0x1fd00220:
		return 0x08521120;
		case 0x0ff00960:
		return 0x100;
		case 0x0ff00160:
		return 1<<24;
		case 0xefdfb000044:
		return random();
		case 0x1fd000c8:
		return 0x10;
		case 0x1ff10000 ... 0x1ff1000b:
		return 0;
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


static void *ls1a_intctl_init(MemoryRegion *mr, hwaddr addr, qemu_irq parent_irq);

static const int sector_len = 32 * 1024;

static PCIBus *pcibus_ls3a2h_init(int busno,qemu_irq *pic, int (*board_map_irq)(PCIDevice *d, int irq_num),MemoryRegion *iomem_axi,  MemoryRegion *ram, ram_addr_t ram_size);



static int ddr2config = 0;

static CPUUnassignedAccess real_do_unassigned_access;
static void mips_ls3a2h_do_unassigned_access(CPUState *cpu, hwaddr addr,
                                           bool is_write, bool is_exec,
                                           int opaque, unsigned size)
{
    if (!is_exec) {
        /* ignore invalid access (ie do not raise exception) */
        return;
    }
    (*real_do_unassigned_access)(cpu, addr, is_write, is_exec, opaque, size);
}

static void ls2h_set_irq(void *opaque, int irq, int level)
{
	CPUMIPSState *env = opaque;
	static int irqstatus = 0;
	if(level) irqstatus |= 1<<irq;
	else irqstatus &= ~(1<<irq);

	qemu_set_irq(env->irq[3],!!irqstatus);
}

#define MAX_CPUS 4
static CPUMIPSState *mycpu[4];

#define CORE0_STATUS_OFF       0x000
#define CORE0_EN_OFF           0x004
#define CORE0_SET_OFF          0x008
#define CORE0_CLEAR_OFF        0x00c
#define CORE0_BUF_20           0x020
#define CORE0_BUF_28           0x028
#define CORE0_BUF_30           0x030
#define CORE0_BUF_38           0x038

#define CORE1_STATUS_OFF       0x100
#define CORE1_EN_OFF           0x104
#define CORE1_SET_OFF          0x108
#define CORE1_CLEAR_OFF        0x10c
#define CORE1_BUF_20           0x120
#define CORE1_BUF_28           0x128
#define CORE1_BUF_30           0x130
#define CORE1_BUF_38           0x138

#define CORE2_STATUS_OFF       0x200
#define CORE2_EN_OFF           0x204
#define CORE2_SET_OFF          0x208
#define CORE2_CLEAR_OFF        0x20c
#define CORE2_BUF_20           0x220
#define CORE2_BUF_28           0x228
#define CORE2_BUF_30           0x230
#define CORE2_BUF_38           0x238

#define CORE3_STATUS_OFF       0x300
#define CORE3_EN_OFF           0x304
#define CORE3_SET_OFF          0x308
#define CORE3_CLEAR_OFF        0x30c
#define CORE3_BUF_20           0x320
#define CORE3_BUF_28           0x328
#define CORE3_BUF_30           0x330
#define CORE3_BUF_38           0x338

#define MYID 0xa0


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
    CPUState *cpu = current_cpu;

    int node = (addr>>44)&3;
	int coreno = (addr>>8)&3;
	int no = coreno+node*4;
    

    if(size!=4) hw_error("size not 4");

//    printf("gipi_writel addr=%llx val=%8x\n", addr, val);
    addr &= 0xff;
    switch(addr){
        case CORE0_STATUS_OFF: 
            hw_error("CORE0_SET_OFF Can't be write\n");
            break;
        case CORE0_EN_OFF:
		if((cpu->mem_io_vaddr&0xff)!=addr) break;
            s->core[no].en = val;
            break;
        case CORE0_SET_OFF:
            s->core[no].status |= val;
            qemu_irq_raise(s->core[no].irq);
            break;
        case CORE0_CLEAR_OFF:
		if((cpu->mem_io_vaddr&0xff)!=addr) break;
            s->core[no].status ^= val;
            qemu_irq_lower(s->core[no].irq);
            break;
        case 0x20 ... 0x3c:
            s->core[no].buf[(addr-0x20)/4] = val;
            break;
        default:
            break;
       }
    DPRINTF("gipi_write: addr=0x%02x val=0x%02x cpu=%d pc=%x\n", addr, val, (int)cpu->cpu_index, (int)mypc);
}

static uint64_t gipi_readl(void *opaque, hwaddr addr, unsigned size)
{
    gipiState * s = opaque;
#ifdef DEBUG_LS3A
    CPUState *cpu = current_cpu;
#endif
    uint32_t ret=0;
    int node = (addr>>44)&3;
	int coreno = (addr>>8)&3;
	int no = coreno+node*4;
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

    //if(ret!=0) printf("CPU#%d:gipi_read: NODE#%d addr=%p val=0x%02x pc=%x, opaque=%p\n",cpu->cpu_index, node, addr, ret, cpu->active_tc.PC, opaque);
    //if(ret!=0) printf("CPU#%d:gipi_read: addr=0x%02x val=0x%02x pc=%x\n",cpu->cpu_index, addr, ret, cpu->PC[0]);
    DPRINTF("gipi_read: addr=0x%02x val=0x%02x cpu=%d pc=%x\n", addr, ret, (int)cpu->cpu_index, (int)mypc);
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


static void mips_ls3a2h_init(MachineState *machine)
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
	ResetData *reset_info[smp_cpus];
	qemu_irq *ls3a2h_irq,*ls3a2h_irq1, *ls2h_irq;
	PCIBus *pci_bus[4];
	DriveInfo *flash_dinfo=NULL;
	CPUClass *cc;
	MemoryRegion *iomem_axi = g_new(MemoryRegion, 1);
	MemoryRegion *iomem_axi1 = g_new(MemoryRegion, 1);
	MemoryRegion *iomem_axi2 = g_new(MemoryRegion, 1);
	MemoryRegion *iomem_axi3 = g_new(MemoryRegion, 1);
	AddressSpace *as = g_new(AddressSpace, 1);
	AddressSpace *as_axi = g_new(AddressSpace, 1);
	int i;

  gipiState * gipis =g_malloc0(sizeof(gipiState));
  gipi_iomem = g_new(MemoryRegion, 1);
  memory_region_init_io(gipi_iomem, NULL, &gipi_ops, (void *)gipis, "gipi", 0x100);


	/* init CPUs */

  for(i = 0; i < smp_cpus; i++) {
    printf("==== init smp_cpus=%d ====\n", i);
	cpu = MIPS_CPU(cpu_create(machine->cpu_type));
		env = &cpu->env;
		mycpu[i] = env;
		env->CP0_EBase |= i;
		env->CP0_Status |= (1 << CP0St_KX);
		env->CP0_PRid   |= 0x6303;

		cc = CPU_GET_CLASS(cpu);
		real_do_unassigned_access = cc->do_unassigned_access;
		cc->do_unassigned_access = mips_ls3a2h_do_unassigned_access;

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

		/* allocate RAM */
	memory_region_init_ram(ram, NULL, "mips_r4k.ram", ram_size, &error_fatal);

	MemoryRegion *ram1 = g_new(MemoryRegion, 1);
	memory_region_init_alias(ram1, NULL, "lowmem", ram, 0, 0x10000000);
	MemoryRegion *ram4 = g_new(MemoryRegion, 1);
	memory_region_init_alias(ram4, NULL, "aximem", ram, 0, ram_size);
	MemoryRegion *ram0 = g_new(MemoryRegion, 1);
	memory_region_init_alias(ram0, NULL, "mem", ram, 0, ram_size);
	memory_region_add_subregion(address_space_mem, 0, ram1);
	memory_region_add_subregion(address_space_mem, 0x80000000ULL, ram0);

        memory_region_init(iomem_axi, OBJECT(address_space_mem),  "ls3a2h axi", UINT64_MAX);
	address_space_init(as_axi, iomem_axi, "ls3a2h axi memory");

	MemoryRegion *iomem_axidmamem = g_new(MemoryRegion, 1);
	memory_region_init(iomem_axidmamem, OBJECT(address_space_mem), "system", INT64_MAX);
	address_space_init(as, iomem_axidmamem, "ls3a2h axi dma memory");

	MemoryRegion *ram_axidmaram = g_new(MemoryRegion, 1);
	memory_region_init_alias(ram_axidmaram, NULL, "aximem", ram, 0, ram_size);
	memory_region_add_subregion(iomem_axidmamem, 0, ram_axidmaram);


	memory_region_init_alias(iomem_axi1, NULL, "iosubmem", iomem_axi, 0x10000000, 0x09000000);
	memory_region_init_alias(iomem_axi2, NULL, "iosubdev", iomem_axi, 0x1f000000, 0x01000000);
	memory_region_init_alias(iomem_axi3, NULL, "iosubbigmem", iomem_axi, 0x40000000, 0x40000000);
	memory_region_add_subregion(address_space_mem, 0x10000000ULL, iomem_axi1);
	memory_region_add_subregion(address_space_mem, 0x1b000000ULL, iomem_axi2);
	memory_region_add_subregion(address_space_mem, 0x40000000ULL, iomem_axi3);
	memory_region_add_subregion(address_space_mem, 0x00000e0000000000ULL, ram4);



	//memory_region_init_iommu(address_space_mem,);
        //memory_region_init_iommu(address_space_mem, OBJECT(dev), &ls3a2h_pciedma_iommu_ops, "iommu-ls3a2hpcie", UINT64_MAX);

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
        reset_info[0]->vector = load_kernel();
    }




	/* Register 64 KB of IO space at 0x1f000000 */
	//isa_mmio_init(0x1ff00000, 0x00010000);
	//isa_mem_base = 0x10000000;
	ls2h_irq = qemu_allocate_irqs(ls2h_set_irq, env, 4);
	ls3a2h_irq =ls1a_intctl_init(iomem_axi, 0x1fd00040, ls2h_irq[0]);
	ls3a2h_irq1=ls1a_intctl_init(iomem_axi, 0x1fd00058, ls2h_irq[1]);
	ls1a_intctl_init(iomem_axi, 0x1fd00070, ls2h_irq[2]);
	ls1a_intctl_init(iomem_axi, 0x1fd00088, ls2h_irq[3]);

    if (serial_hd(0))
            serial_mm_init(address_space_mem, 0x1fe001e0, 0,env->irq[2],115200,serial_hd(0), DEVICE_NATIVE_ENDIAN);

	if (serial_hd(1))
		serial_mm_init(iomem_axi, 0x1fe80000, 0,ls3a2h_irq[2],115200,serial_hd(1), DEVICE_NATIVE_ENDIAN);

	if (serial_hd(2))
		serial_mm_init(iomem_axi, 0x1fe81000, 0,ls3a2h_irq[3],115200,serial_hd(2), DEVICE_NATIVE_ENDIAN);

	if (serial_hd(3))
		serial_mm_init(iomem_axi, 0x1fe82000, 0,ls3a2h_irq[4],115200,serial_hd(3), DEVICE_NATIVE_ENDIAN);

	if (serial_hd(4))
		serial_mm_init(iomem_axi, 0x1fe83000, 0,ls3a2h_irq[5],115200,serial_hd(4), DEVICE_NATIVE_ENDIAN);

	DeviceState *dev;
	dev = sysbus_create_simple("ls2h_fb", -1, NULL);
	sysbus_mmio_map_iomem(iomem_axi, SYS_BUS_DEVICE(dev), 0, 0x1fe50000);
	qdev_prop_set_ptr(dev, "root", iomem_axidmamem);



	pci_bus[0]=pcibus_ls3a2h_init(0, &ls3a2h_irq1[20],pci_ls3a2h_map_irq, iomem_axi, ram, ram_size);
	pci_bus[1]=pcibus_ls3a2h_init(1, &ls3a2h_irq1[21],pci_ls3a2h_map_irq, iomem_axi, ram, ram_size);
	pci_bus[2]=pcibus_ls3a2h_init(2, &ls3a2h_irq1[22],pci_ls3a2h_map_irq, iomem_axi, ram, ram_size);
	pci_bus[3]=pcibus_ls3a2h_init(3, &ls3a2h_irq1[23],pci_ls3a2h_map_irq, iomem_axi, ram, ram_size);


	{
		DeviceState *dev;
		dev = qdev_create(NULL, "exynos4210-ehci-usb");
		qdev_prop_set_ptr(dev, "as", as);
		qdev_init_nofail(dev);

		sysbus_mmio_map_iomem(iomem_axi,SYS_BUS_DEVICE(dev), 0, 0x1fe00000);

		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls3a2h_irq1[0]);

	}
	{
		DeviceState *dev;
		dev = qdev_create(NULL, "sysbus-ohci");
		qdev_prop_set_uint32(dev, "num-ports", 4);
		qdev_prop_set_uint64(dev, "dma-offset", 0);
		qdev_init_nofail(dev);
		qdev_prop_set_ptr(dev, "as", as);
		sysbus_mmio_map_iomem(iomem_axi, SYS_BUS_DEVICE(dev), 0, 0x1fe08000);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls3a2h_irq1[1]);

	}

	{
		DeviceState *dev;
		BusState *idebus[4];
		DriveInfo *hd;
		dev = qdev_create(NULL, "sysbus-ahci");
		qdev_prop_set_uint32(dev, "num-ports", 1);
		qdev_prop_set_ptr(dev, "as", as);
		qdev_init_nofail(dev);
		sysbus_mmio_map_iomem(iomem_axi,SYS_BUS_DEVICE(dev), 0, 0x1fe30000);
		sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls3a2h_irq1[5]);
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



#if 0
	sysbus_create_simple("ls3a2h_acpi",0x1fe7c000, ls3a2h_irq[0]);
#endif

	if (nb_nics) {
#if 0
	int i;
	int devfn;
		//gmac_sysbus_create(&nd_table[0], 0x1fe10000, ls3a2h_irq1[3]);
	   for(i=0,devfn=PCI_DEVFN(2, 0);i<nb_nics;i++,devfn += PCI_DEVFN(1, 0))
	  {
		PCIDevice *pci_dev;
		DeviceState *dev;
            //dev = pci_nic_init(&nd_table[i],nd_table[i].model?:"e1000","01:0b");
		pci_dev = pci_create(pci_bus[i], -1, "e1000");
		dev = &pci_dev->qdev;
		qdev_set_nic_properties(dev, &nd_table[i]);
    		DeviceClass *dc = DEVICE_GET_CLASS(dev);
    		PCIDeviceClass *k = PCI_DEVICE_CLASS(DEVICE_CLASS(dc));
    		k->romfile = NULL;
    		dc->vmsd = NULL;
		qdev_init_nofail(dev);

	    	printf("nb_nics=%d dev=%p\n", nb_nics, pci_dev);
	  }
#else
		printf("pci_bus=%p\n", pci_bus);
		{
			DeviceState *dev;

			dev = qdev_create(NULL, "sysbus-synopgmac");
			qdev_set_nic_properties(dev, &nd_table[0]);
			qdev_prop_set_ptr(dev, "as", as);
			qdev_prop_set_int32(dev, "enh_desc", 1);
			qdev_init_nofail(dev);
			sysbus_mmio_map_iomem(iomem_axi, SYS_BUS_DEVICE(dev), 0, 0x1fe10000);
			sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls3a2h_irq1[3]);
		}
		{
			DeviceState *dev;

			dev = qdev_create(NULL, "sysbus-synopgmac");
			qdev_set_nic_properties(dev, &nd_table[1]);
			qdev_prop_set_ptr(dev, "as", as);
			qdev_prop_set_int32(dev, "enh_desc", 1);
			qdev_init_nofail(dev);
			sysbus_mmio_map_iomem(iomem_axi, SYS_BUS_DEVICE(dev), 0, 0x1fe18000);
			sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, ls3a2h_irq1[4]);
		}
#endif
	}

#if 0
	{
		sysbus_create_simple("ls3a2h_wdt", 0x1fe7c060, NULL);
	}
#endif

	{
		DeviceState *dev,*dev1;
		void *bus;
		qemu_irq cs_line;
		dev=sysbus_create_simple("ls1a_spi",0x1fe70000, ls3a2h_irq[8]);
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
		dev = qdev_create(NULL, "ls3a2h_ac97");
		printf("ac97 dev=%p\n",dev);
		qdev_init_nofail(dev);
		s = SYS_BUS_DEVICE(dev);
		sysbus_mmio_map(s, 0, 0x1fe74000);
		sysbus_connect_irq(s, 0, ls3a2h_irq[14]);
		sysbus_connect_irq(s, 1, ls3a2h_irq[15]);
	}
#endif

		dev = sysbus_create_simple("ls1a_dma", -1, NULL);
		sysbus_mmio_map_iomem(iomem_axi, SYS_BUS_DEVICE(dev), 0, 0x1fd00100);

		dev = sysbus_create_simple("ls1a_nand", -1, ls3a2h_irq[13]);
		sysbus_mmio_map_iomem(iomem_axi, SYS_BUS_DEVICE(dev), 0, 0x1fee0000);
		qdev_prop_set_ptr(dev, "as", as);


#if 1
	{
		DeviceState *dev;
		SysBusDevice *s;
		hwaddr devaddr =  0x1fef8020;
		dev=sysbus_create_varargs("ls1a_rtc", -1, ls3a2h_irq[14], ls3a2h_irq[15], ls3a2h_irq[16], NULL);
		s =  SYS_BUS_DEVICE(dev);
		s->mmio[0].addr = devaddr;
		memory_region_add_subregion(iomem_axi, devaddr, s->mmio[0].memory);
	}
#endif

	{
		DeviceState *dev;
		void *bus;
		dev=sysbus_create_simple("ls1a_i2c", -1, ls3a2h_irq[7]);
		sysbus_mmio_map_iomem(iomem_axi, SYS_BUS_DEVICE(dev), 0, 0x1fe90000);
		bus = qdev_get_child_bus(dev, "i2c");
		i2c_create_slave(bus, "ds1338", 0x68);
	}

	{
		DeviceState *dev;
		void *bus;
		dev=sysbus_create_simple("ls1a_i2c", -1, ls3a2h_irq[8]);
		sysbus_mmio_map_iomem(iomem_axi, SYS_BUS_DEVICE(dev), 0, 0x1fe91000);
		bus = qdev_get_child_bus(dev, "i2c");
		i2c_create_slave(bus, "ds1338", 0x68);
	}


	{

                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fd00210, "0x1fd00210", 0x4);
                memory_region_add_subregion(iomem_axi, 0x1fd00210, iomem);
		/*ins*/
                iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fd00800, "0x1fd00800", 0x34);
                memory_region_add_subregion(iomem_axi, 0x1fd00800, iomem);

		/*gpio*/
                iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fd000c8, "0x1fd000c8", 0x8);
                memory_region_add_subregion(iomem_axi, 0x1fd000c8, iomem);
		/*lpc*/
                iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1ff10000, "0x1ff10000", 0x12);
                memory_region_add_subregion(iomem_axi, 0x1ff10000, iomem);
	}


	{
                ddrcfg_iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(ddrcfg_iomem, NULL, &mips_qemu_ops, (void *)0x0ff00000, "ddr", 0x100000);
	}

	{

                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0x1fe00180, "0x1fe00180", 0x8);
                memory_region_add_subregion(address_space_mem, 0x1fe00180, iomem);
		mips_qemu_writel((void *)0x1fe00180, 0, 0xff003180, 4);
	}

	{

                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &mips_qemu_ops, (void *)0xefdfb000000, "0xefdfb000000", 0x100);
                memory_region_add_subregion(address_space_mem, 0xefdfb000000, iomem);
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
		sysbus_connect_irq(s, 0, ls3a2h_irq1[25]);
		hdabus = QLIST_FIRST(&dev->child_bus);
		codec = qdev_create(hdabus, "hda-duplex");
		qdev_init_nofail(codec);
	}
#endif

}


static void mips_machine_init(MachineClass *mc)
{
    mc->desc = "mips ls3a2h platform";
    mc->init = mips_ls3a2h_init;
    mc->max_cpus = 16;
    mc->block_default_type = IF_IDE;
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("godson3");
}

DEFINE_MACHINE("ls3a2h", mips_machine_init)

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

#undef DPRINTF
#include "ls1a_int.c"
//-------------------------
// pci bridge
//-----------------

static int pci_ls3a2h_map_irq(PCIDevice *d, int irq_num)
{
	return irq_num;
}

static void pci_ls3a2h_set_irq(void *opaque, int irq_num, int level)
{
qemu_irq *pic = opaque;
	qemu_set_irq(pic[irq_num],level);
}
/*self pci header*/
#define LS2H_PCIE_PORT_HEAD_BASE_PORT(portnum)  (0x18114000 + (portnum << 22))
/*devices pci header*/
#define LS3H_PCIE_DEV_HEAD_BASE_PORT(portnum)   (0x18116000 + (portnum << 22))
/*pci map*/
#define LS2H_PCIE_REG_BASE_PORT(portnum)        (0x18118000 + (portnum << 22))
#define LS2H_PCIE_PORT_REG_STAT1		0xC
#define LS2H_PCIE_PORT_REG_CTR0			0x0
#define LS2H_PCIE_PORT_REG_CFGADDR		0x24
#define LS2H_PCIE_PORT_REG_CTR_STAT		0x28


#define TYPE_BONITO_PCI_HOST_BRIDGE "ls3a2h-pcihost"
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
    PCIExpressHost parent_obj;
    PCIBus *bus;
    qemu_irq *pic;
    PCIBonitoState *pci_dev;
    MemoryRegion iomem_mem;
    MemoryRegion iomem_submem;
    MemoryRegion iomem_subbigmem;
    MemoryRegion iomem_io;
    AddressSpace as_mem;
    AddressSpace as_dmamem;
    MemoryRegion iomem_dmamem;
    AddressSpace as_io;
   int (*pci_map_irq)(PCIDevice *d, int irq_num);
};

static inline uint32_t bonito_pci_config_addr(PCIBonitoState *s,hwaddr addr)
{
	int bus = 0, dev = -1, func = 0, reg = 0;
	uint32_t busaddr;
	struct pcilocalreg *d=&s->mypcilocalreg;
	busaddr = d->portcfg;

	bus = busaddr >> 16;
	dev = (busaddr >> 11) & 0x1f;
	func = (busaddr >> 8) & 7;
	reg = (addr & 0xfc);

	return bus<<16|dev<<11|func<<8|reg;
}

static void pci_ls3a2h_config_writel (void *opaque, hwaddr addr,
		uint64_t val, unsigned size)
{
	PCIBonitoState *s = opaque;
	//   PCIDevice *d = PCI_DEVICE(s);
	BonitoState *phb = s->pcihost;
	pci_data_write(phb->bus, bonito_pci_config_addr(s, addr), val, size);
}

static uint64_t pci_ls3a2h_config_readl (void *opaque, hwaddr addr, unsigned size)
{
	PCIBonitoState *s = opaque;
	//  PCIDevice *d = PCI_DEVICE(s);
	BonitoState *phb = s->pcihost;
	uint32_t val;
	val = pci_data_read(phb->bus, bonito_pci_config_addr(s, addr), size);
	return val;
}


static const MemoryRegionOps pci_ls3a2h_config_ops = {
    .read = pci_ls3a2h_config_readl,
    .write = pci_ls3a2h_config_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


#define LS2H_PCIE_REG_CTR0_BIT_LTSSM_EN			(1 << 3)
#define LS2H_PCIE_REG_CTR0_BIT_REQ_L1			(1 << 12)
#define LS2H_PCIE_REG_CTR0_BIT_RDY_L23			(1 << 13)
#define LS2H_PCIE_PORT_REG_STAT1		0xC
#define LS2H_PCIE_REG_STAT1_MASK_LTSSM		0x0000003f
#define LS2H_PCIE_REG_STAT1_BIT_LINKUP			(1 << 6)
#define LS2H_PCIE_PORT_REG_CFGADDR		0x24
#define LS2H_PCIE_PORT_REG_CTR_STAT		0x28
#define LS2H_PCIE_REG_CTR_STAT_BIT_ISX4			(1 << 26)
#define LS2H_PCIE_REG_CTR_STAT_BIT_ISRC			(1 << 27)

static uint64_t pci_bonito_local_readl (void *opaque, hwaddr addr, unsigned size)
{
    PCIBonitoState *s = opaque;
	uint32_t val;

	//printf("local addr=%x\n", (unsigned int)addr);
	if(addr>=sizeof(struct pcilocalreg)) return 0;

	switch(addr)
	{
	case LS2H_PCIE_PORT_REG_CTR_STAT:
	 val = LS2H_PCIE_REG_CTR_STAT_BIT_ISRC|LS2H_PCIE_REG_STAT1_BIT_LINKUP;
         break;
	case LS2H_PCIE_PORT_REG_STAT1:
	 val = -1;
	 break;
	default:
	 val = ((uint32_t *)&s->mypcilocalreg)[addr/sizeof(uint32_t)];
	 break;
	}
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
	//printf("local addr=%x\n", (unsigned int)addr);
	if(addr>=sizeof(struct pcilocalreg)) return;
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

static void bonito_initfn(PCIDevice *dev, Error **errp)
{
    PCIBonitoState *s = DO_UPCAST(PCIBonitoState, parent_obj.parent_obj, dev);
    SysBusDevice *sysbus = SYS_BUS_DEVICE(s->pcihost);

    pci_bridge_initfn(dev, TYPE_PCI_BUS);

    memory_region_init_io(&s->iomem, NULL, &pci_bonito_local_ops, s, "ls3a2h_pci_conf", 0x100);
    sysbus_init_mmio(sysbus, &s->iomem);

    /* set the north bridge pci configure  mapping */
    memory_region_init_io(&s->conf_mem, NULL, &bonito_pciconf_ops, s,
                          "north-bridge-pci-config", 0x100);
    sysbus_init_mmio(sysbus, &s->conf_mem);

    /* set the south bridge pci configure  mapping */
    memory_region_init_io(&s->data_mem, NULL, &pci_ls3a2h_config_ops, s,
                          "south-bridge-pci-config", 0x100);
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
    .name          = "LS3A2H_Bonito",
    .parent        = TYPE_PCI_BRIDGE,
    .instance_size = sizeof(PCIBonitoState),
    .class_init    = bonito_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { }
    },
};

static AddressSpace *pci_dma_context_fn(PCIBus *bus, void *opaque, int devfn);

static PCIBus *pcibus_ls3a2h_init(int busno, qemu_irq *pic, int (*board_map_irq)(PCIDevice *d, int irq_num), MemoryRegion *iomem_axi, MemoryRegion *ram, ram_addr_t ram_size)
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
    //d = pci_create(phb->bus, PCI_DEVFN(0, 0), "LS3A2H_Bonito");
    d = pci_create_multifunction(pcihost->bus, PCI_DEVFN(0, 0), true, "LS3A2H_Bonito");

    s = DO_UPCAST(PCIBonitoState, parent_obj.parent_obj, d);
    s->pcihost = pcihost;
    pcihost->pci_dev = s;
    br = PCI_BRIDGE(d);
    pci_bridge_map_irq(br, "Advanced PCI Bus secondary bridge 1", board_map_irq);
    qdev_init_nofail(DEVICE(d));
    bus2 = pci_bridge_get_sec_bus(br);

    pci_setup_iommu(bus2, pci_dma_context_fn, pcihost);


    sysbus = SYS_BUS_DEVICE(s->pcihost);
     /*local map*/
    sysbus_mmio_map_iomem(iomem_axi, sysbus, 0, LS2H_PCIE_REG_BASE_PORT(busno));
     /*self header*/
    sysbus_mmio_map_iomem(iomem_axi, sysbus, 1, LS2H_PCIE_PORT_HEAD_BASE_PORT(busno));
     /*devices header*/
    sysbus_mmio_map_iomem(iomem_axi, sysbus, 2, LS3H_PCIE_DEV_HEAD_BASE_PORT(busno));

    memory_region_init_alias(&pcihost->iomem_submem, NULL, "pcisubmem", &pcihost->iomem_mem, 0x10000000UL+busno*0x2000000UL, 0x2000000UL);
    memory_region_init_alias(&pcihost->iomem_subbigmem, NULL, "pcisubbigmem", &pcihost->iomem_mem, 0x40000000UL+busno*0x10000000UL, 0x10000000UL);
    memory_region_add_subregion(iomem_axi, 0x10000000UL+busno*0x2000000UL, &pcihost->iomem_submem);
    memory_region_add_subregion(iomem_axi, 0x40000000UL+busno*0x10000000UL, &pcihost->iomem_subbigmem);
    memory_region_add_subregion(iomem_axi, 0x18100000UL+busno*0x400000UL, &pcihost->iomem_io);


    MemoryRegion *ram_pciram = g_new(MemoryRegion, 1);
    memory_region_init_alias(ram_pciram, NULL, "aximem", ram, 0, ram_size);
    memory_region_add_subregion(&pcihost->iomem_dmamem, 0x00000000UL, ram_pciram);

    return bus2;
}


static AddressSpace *pci_dma_context_fn(PCIBus *bus, void *opaque, int devfn)
{
    BonitoState *pcihost = opaque;
    return &pcihost->as_dmamem;
}


/* Handle PCI-to-system address translation.  */
/* TODO: A translation failure here ought to set PCI error codes on the
   Pchip and generate a machine check interrupt.  */

static int bonito_pcihost_initfn(SysBusDevice *dev)
{
    BonitoState *pcihost;
    pcihost = BONITO_PCI_HOST_BRIDGE(dev);

    memory_region_init(&pcihost->iomem_mem, OBJECT(pcihost), "system", INT64_MAX);
    address_space_init(&pcihost->as_mem, &pcihost->iomem_mem, "pcie memory");

    memory_region_init(&pcihost->iomem_dmamem, OBJECT(pcihost), "system", INT64_MAX);
    address_space_init(&pcihost->as_dmamem, &pcihost->iomem_dmamem, "pcie memory");

    /* Host memory as seen from the PCI side, via the IOMMU.  */
//    memory_region_init_iommu(&pcihost->iomem_mem, OBJECT(dev), &ls3a2h_pciedma_iommu_ops, "iommu-ls3a2hpcie", UINT64_MAX);


    memory_region_init(&pcihost->iomem_io, OBJECT(pcihost), "system", 0x10000);
    address_space_init(&pcihost->as_io, &pcihost->iomem_io, "pcie io");

    pcihost->bus = pci_register_root_bus(DEVICE(dev), "pci",
                                pci_ls3a2h_set_irq, pcihost->pci_map_irq, pcihost->pic,
                                &pcihost->iomem_mem, &pcihost->iomem_io,
                                PCI_DEVFN(10, 0), 4, TYPE_PCI_BUS);

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
#define LOONGSON_3A2H
#define loongson3A3

#include "loongson_bootparam.c"
