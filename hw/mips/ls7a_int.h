#ifndef __LS7A_INT_H__
#define __LS7A_INT_H__
typedef struct LS7A_INTCTLState {
	uint64_t baseaddr;
	uint64_t int_pol;
	uint64_t int_mask;
	uint64_t htmsi_en;
	uint64_t int_edge;
	uint64_t int_pending;
	uint64_t soft_int;
	uint64_t int_auto_ctrl0;
	uint64_t int_auto_ctrl1;
	uint64_t intreg_pending;
	char irqroute[64];
	char msiroute[64];
#ifdef DEBUG_IRQ_COUNT
	uint64_t irq_count[32];
#endif
	qemu_irq cpu_irq;
	uint64_t pil_out;
	qemu_irq *irqs;
	uint64_t route_int[2];
	uint32_t msiroute_ht[8];
} LS7A_INTCTLState;

static LS7A_INTCTLState *ls7a_intctl_init(MemoryRegion *mr, hwaddr addr, qemu_irq parent_irq);
#endif
