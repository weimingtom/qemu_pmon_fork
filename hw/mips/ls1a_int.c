//#define DEBUG_IRQ

#ifdef DEBUG_IRQ
#define DPRINTF(fmt, args...) \
	do { printf("IRQ: " fmt , ##args); } while (0)
#else
#define DPRINTF(fmt, args...)
#endif

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

static void ls1a_check_interrupts(void *opaque);

// per-cpu interrupt controller
static uint64_t ls1a_intctl_mem_readl(void *opaque, hwaddr addr, unsigned size)
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

static void ls1a_intctl_mem_writel(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
	GS232_INTCTLState *s = opaque;
	uint32_t saddr;

	saddr = addr >> 2;
	//printf("write reg 0x" TARGET_FMT_plx " %x= %x\n", addr, saddr, (unsigned int)val);
	switch (saddr) {
		case 0: //isr
			s->intreg_pending=val;
			ls1a_check_interrupts(s);
			break;
		case 1:
			s->intreg_en=val;
			ls1a_check_interrupts(s);
			break;
		case 2: //set
			s->intreg_en |= val;
			ls1a_check_interrupts(s);
			break;
		case 3: //clr
			s->intreg_pending &= ~(val & s->intreg_edge);
			ls1a_check_interrupts(s);
			break;
		case 4:
			s->intreg_pol=val;
			ls1a_check_interrupts(s);
			break;
		case 5:
			s->intreg_edge=val;
			ls1a_check_interrupts(s);
			break;
		default:
			break;
	}
}

static const MemoryRegionOps ls1a_intctl_mem_ops = {
    .read = ls1a_intctl_mem_readl,
    .write = ls1a_intctl_mem_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static void ls1a_check_interrupts(void *opaque)
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
static void ls1a_set_irq(void *opaque, int irq, int level)
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
	ls1a_check_interrupts(s);
}



static void ls1a_intctl_reset(void *opaque)
{
	GS232_INTCTLState *s = opaque;

	s->intreg_pending = 0;
	ls1a_check_interrupts(s);
}


static void *ls1a_intctl_init(MemoryRegion *mr, hwaddr addr, qemu_irq parent_irq)
{
	qemu_irq *irqs;
	GS232_INTCTLState *s;

	s = g_malloc0(sizeof(GS232_INTCTLState));
	if (!s)
		return NULL;

	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &ls1a_intctl_mem_ops, s, "ls1a_int", INTCTL_SIZE);
                memory_region_add_subregion(mr, addr, iomem);
	}

	s->cpu_irq = parent_irq;
	s->baseaddr=addr;

	qemu_register_reset(ls1a_intctl_reset, s);
	irqs = qemu_allocate_irqs(ls1a_set_irq, s, 32);

	ls1a_intctl_reset(s);
	return irqs;
}
