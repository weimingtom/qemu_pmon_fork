#include "ls7a_int.h"
//#define DEBUG_IRQ

#ifdef DEBUG_IRQ
#define DPRINTF(fmt, args...) \
	do { printf("IRQ: " fmt , ##args); } while (0)
#else
#define DPRINTF(fmt, args...)
#endif

#define INTCTL_SIZE 0x400
#define INTCTLM_MAXADDR 0x13
#define INTCTLM_SIZE (INTCTLM_MAXADDR + 1)
#define INTCTLM_MASK 0x1f
#define MASTER_IRQ_MASK ~0x0fa2007f
#define MASTER_DISABLE 0x80000000
#define CPU_SOFTIRQ_MASK 0xfffe0000
#define CPU_HARDIRQ_MASK 0x0000fffe
#define CPU_IRQ_INT15_IN 0x0004000
#define CPU_IRQ_INT15_MASK 0x80000000

static void ls7a_check_interrupts(void *opaque);

// per-cpu interrupt controller
static uint64_t ls7a_intctl_mem_readl(void *opaque, hwaddr addr, unsigned size)
{
	LS7A_INTCTLState *s = opaque;
	uint64_t ret = 0, val = 0;

	switch (addr) {
		case 0:
			ret = 0x003f0001;
		break;
		case 0x20 ... 0x27:
			memcpy(&ret, ((char *)&s->int_mask)+(addr-0x20), size);
			break;
		case 0x40 ... 0x47:
			memcpy(&ret, ((char *)&s->htmsi_en)+(addr-0x40), size);
			break;
		case 0x60 ... 0x67:
			memcpy(&ret, ((char *)&s->int_edge)+(addr-0x60), size);
			break;
		case 0x80 ... 0x87: //int_clr
			return 0;
		case 0xa0 ... 0xa7:
			memcpy(&ret, ((char *)&s->soft_int)+(addr-0xa0), size);
			break;
		case 0xc0 ... 0xc7:
			memcpy(&ret, ((char *)&s->int_auto_ctrl0)+(addr-0xc0), size);
			break;
		case 0xe0 ... 0xe7:
			memcpy(&ret, ((char *)&s->int_auto_ctrl1)+(addr-0xe0), size);
			break;
		case 0x100 ... 0x13f:
			memcpy(&ret, s->irqroute+addr-0x100, size);
			break;
		case 0x200 ... 0x23f:
			memcpy(&ret, s->msiroute+addr-0x200, size);
			break;
		case 0x300 ... 0x30f: //intn0 isr
			val = s->intreg_pending & ~s->int_mask;
			memcpy(&ret, (char *)&val+(addr-0x300), size);
			break;
		case 0x320 ... 0x32f: //intn1 isr
			val  = s->intreg_pending & ~s->int_mask;
			memcpy(&ret, (char *)&val+(addr-0x320), size);
			break;
		case 0x380 ... 0x38f:
			memcpy(&ret, (char *)&s->intreg_pending + addr-0x380, size);
			break;
		case 0x3a0 ... 0x3af:
			val = s->intreg_pending & ~s->int_mask;
			memcpy(&ret, (char *)&val+(addr-0x3a0), size);
			break;
		case 0x3e0 ... 0x3ef:
			memcpy(&ret, (char *)&s->int_pol + (addr - 0x3e0), size);
			break;
		default:
			ret = 0;
			break;
	}
	DPRINTF("read reg 0x" TARGET_FMT_plx " = %lx\n", addr, ret);

	return ret;
}

static void ls7a_intctl_mem_writel(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
	LS7A_INTCTLState *s = opaque;
	//uint32_t saddr;
	int i, j;

	DPRINTF("write reg 0x" TARGET_FMT_plx " = %x\n", addr, (unsigned int)val);
	switch (addr) {
		case 0x20 ... 0x27:
			memcpy(((char *)&s->int_mask)+(addr-0x20), &val, size);
			break;
		case 0x40 ... 0x47:
			memcpy(((char *)&s->htmsi_en)+(addr-0x40), &val, size);
			break;
		case 0x60 ... 0x67:
			memcpy(((char *)&s->int_edge)+(addr-0x60), &val, size);
			break;
		case 0x80 ... 0x87: //int_clr
			for(i=addr - 0x80, j = 0;j<size;i++, j++)
				((char *)&s->int_mask)[i] &= ~((char *)&val)[j];
	                ls7a_check_interrupts(s);
			break;
		case 0xa0 ... 0xa7:
			memcpy(((char *)&s->soft_int)+(addr-0xa0), &val, size);
			break;
		case 0xc0 ... 0xc7:
			memcpy(((char *)&s->int_auto_ctrl0)+(addr-0xc0), &val, size);
			break;
		case 0xe0 ... 0xe7:
			memcpy(((char *)&s->int_auto_ctrl1)+(addr-0xe0), &val, size);
			break;
		case 0x100 ... 0x13f:
			memcpy(s->irqroute+addr-0x100, &val, size);
			if (val == 1)
				s->route_int[0] |= 1ULL<<(addr-0x100); 
			else if(val == 2)
				s->route_int[1] |= 1ULL<<(addr-0x100); 
			else {
				s->route_int[0] &= ~(1ULL<<(addr-0x100)); 
				s->route_int[1] &= ~(1ULL<<(addr-0x100)); 
			}
			break;
		case 0x200 ... 0x23f:
			j = addr-0x200;
			memcpy(s->msiroute + j, &val, size);
			break;
		case 0x300 ... 0x30f: //intn0 isr
			break;
		case 0x320 ... 0x32f: //intn1 isr
			break;
		case 0x380 ... 0x38f:
			break;
		case 0x3a0 ... 0x3af:
			break;
		case 0x3e0 ... 0x3ef:
			break;
		default:
			break;
	}
}

static const MemoryRegionOps ls7a_intctl_mem_ops = {
    .read = ls7a_intctl_mem_readl,
    .write = ls7a_intctl_mem_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static void ls7a_check_interrupts(void *opaque)
{
	LS7A_INTCTLState *s = opaque;
	uint64_t pil_pending;
	pil_pending = s->intreg_pending & ~s->int_mask;

	if (pil_pending) {
		qemu_irq_raise(s->cpu_irq);
	} else {
		if (s->pil_out)
			qemu_irq_lower(s->cpu_irq);
	}

	s->pil_out = pil_pending;
}

/*
 * "irq" here is the bit number in the system interrupt register to
 * separate serial and keyboard interrupts sharing a level.
 */
static void ls7a_set_irq(void *opaque, int irq, int level)
{
	LS7A_INTCTLState *s = opaque;
	uint64_t mask = 1ULL << irq;

	DPRINTF("Set irq %d level %d\n", irq, level);
	if (level) {
		s->intreg_pending |= mask;
	} else {
		s->intreg_pending &= ~mask;
	}
	ls7a_check_interrupts(s);
}



static void ls7a_intctl_reset(void *opaque)
{
	LS7A_INTCTLState *s = opaque;

	s->intreg_pending = 0;
	ls7a_check_interrupts(s);
}


static LS7A_INTCTLState *ls7a_intctl_init(MemoryRegion *mr, hwaddr addr, qemu_irq parent_irq)
{
	LS7A_INTCTLState *s;

	s = g_malloc0(sizeof(LS7A_INTCTLState));
	if (!s)
		return NULL;

	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &ls7a_intctl_mem_ops, s, "ls7a_int", INTCTL_SIZE);
                memory_region_add_subregion(mr, addr, iomem);
	}

	s->cpu_irq = parent_irq;
	s->baseaddr=addr;

	qemu_register_reset(ls7a_intctl_reset, s);
	s->irqs = qemu_allocate_irqs(ls7a_set_irq, s, 64);

	ls7a_intctl_reset(s);
	return s;
}
