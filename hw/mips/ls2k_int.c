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
	char irqroute[32];
#ifdef DEBUG_IRQ_COUNT
	uint64_t irq_count[32];
#endif
	qemu_irq *cpu_irq;
	uint32_t pil_out[4];
} GS232_INTCTLState;

#define INTCTL_SIZE 0x40
#define INTCTLM_MAXADDR 0x13
#define INTCTLM_SIZE (INTCTLM_MAXADDR + 1)
#define INTCTLM_MASK 0x1f
#define MASTER_IRQ_MASK ~0x0fa2007f
#define MASTER_DISABLE 0x80000000
#define CPU_SOFTIRQ_MASK 0xfffe0000
#define CPU_HARDIRQ_MASK 0x0000fffe
#define CPU_IRQ_INT15_IN 0x0004000
#define CPU_IRQ_INT15_MASK 0x80000000

static void ls2k_check_interrupts(void *opaque);

// per-cpu interrupt controller
static uint64_t ls2k_intctl_mem_readl(void *opaque, hwaddr addr, unsigned size)
{
	GS232_INTCTLState *s = opaque;
	uint32_t ret = 0;

	switch (addr) {
		case 0 ... 0x1f:
			memcpy(&ret, s->irqroute+addr, size);
			break;
		case 0x20: //isr
			ret = s->intreg_pending & s->intreg_en;
			break;
		case 0x24:
			ret= s->intreg_en;
			break;
		case 0x28: //set
			ret=0;
			break;
		case 0x2c: //clr
			ret=0;
			break;
		case 0x30:
			ret= s->intreg_pol;
			break;
		case 0x34:
			ret= s->intreg_edge;
			break;
		default:
			ret = 0;
			break;
	}
	DPRINTF("read cpu %d reg 0x" TARGET_FMT_plx " = %x\n", cpu, addr, ret);

	return ret;
}

static void ls2k_intctl_mem_writel(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
	GS232_INTCTLState *s = opaque;
	uint32_t saddr;

	//printf("write reg 0x" TARGET_FMT_plx " %x= %x\n", addr, saddr, (unsigned int)val);
	switch (addr) {
		case 0 ... 0x1f:
			memcpy(s->irqroute +addr, &val, size);
			ls2k_check_interrupts(s);
			break;
		case 0x20: //isr
			s->intreg_pending=val;
			ls2k_check_interrupts(s);
			break;
		case 0x24:
			s->intreg_en=val;
			ls2k_check_interrupts(s);
			break;
		case 0x28: //set
			s->intreg_en |= val;
			ls2k_check_interrupts(s);
			break;
		case 0x2c: //clr
			s->intreg_pending &= ~val;
			s->intreg_en &= ~val;
			ls2k_check_interrupts(s);
			break;
		case 0x30:
			s->intreg_pol=val;
			ls2k_check_interrupts(s);
			break;
		case 0x3c:
			s->intreg_edge=val;
			ls2k_check_interrupts(s);
			break;
		default:
			break;
	}
}

static const MemoryRegionOps ls2k_intctl_mem_ops = {
    .read = ls2k_intctl_mem_readl,
    .write = ls2k_intctl_mem_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static void ls2k_check_interrupts(void *opaque)
{
	GS232_INTCTLState *s = opaque;
	uint32_t pil_pending[4] = {0,0,0,0};
	int i, j, imask, jmask;
	int irqs, maps;
	irqs = s->intreg_pending & s->intreg_en;
	

	for (i=0, imask=1;i<32 && irqs;i++,imask <<= 1)
	{
		if (irqs & imask)
		{
			maps = s->irqroute[i];
			for(j=0, jmask=0x10;j<4 && maps;j++,jmask <<= 1)
			{
				if(maps & jmask)
				{
					pil_pending[j] = 1;
					maps &= ~jmask;
				}
			}
		  irqs &= ~imask;
		}
	}

	for(i=0;i<4;i++)
	{
		if (pil_pending[i]) {
			qemu_irq_raise(s->cpu_irq[i+2]);
		} else {
			if (s->pil_out[i])
				qemu_irq_lower(s->cpu_irq[i+2]);
		}

		s->pil_out[i] = pil_pending[i];
	}
}

/*
 * "irq" here is the bit number in the system interrupt register to
 * separate serial and keyboard interrupts sharing a level.
 */
static void ls2k_set_irq(void *opaque, int irq, int level)
{
	GS232_INTCTLState *s = opaque;
	uint32_t mask = 1 << irq;

	DPRINTF("Set cpu %d irq %d level %d\n", s->target_cpu, irq,
			level);
	if (level) {
		s->intreg_pending |= mask;
	} else {
		s->intreg_pending &= ~mask;
	}
	ls2k_check_interrupts(s);
}


static void ls2k_intctl_save(QEMUFile *f, void *opaque)
{
	GS232_INTCTLState *s = opaque;

	qemu_put_be32s(f, &s->intreg_pending);
}

static int ls2k_intctl_load(QEMUFile *f, void *opaque, int version_id)
{
	GS232_INTCTLState *s = opaque;

	if (version_id != 1)
		return -EINVAL;

	qemu_get_be32s(f, &s->intreg_pending);
	ls2k_check_interrupts(s);
	return 0;
}

static void ls2k_intctl_reset(void *opaque)
{
	GS232_INTCTLState *s = opaque;

	s->intreg_pending = 0;
	ls2k_check_interrupts(s);
}


static void *ls2k_intctl_init(MemoryRegion *mr, hwaddr addr, qemu_irq *parent_irq)
{
	qemu_irq *irqs;
	GS232_INTCTLState *s;

	s = g_malloc0(sizeof(GS232_INTCTLState));
	if (!s)
		return NULL;

	{
                MemoryRegion *iomem = g_new(MemoryRegion, 1);
                memory_region_init_io(iomem, NULL, &ls2k_intctl_mem_ops, s, "ls2k_int", INTCTL_SIZE);
                memory_region_add_subregion(mr, addr, iomem);
	}

	s->cpu_irq = parent_irq;
	s->baseaddr=addr;

	register_savevm(NULL, "ls2k_intctl", addr, 1, ls2k_intctl_save, ls2k_intctl_load, s);
	qemu_register_reset(ls2k_intctl_reset, s);
	irqs = qemu_allocate_irqs(ls2k_set_irq, s, 32);

	ls2k_intctl_reset(s);
	return irqs;
}
