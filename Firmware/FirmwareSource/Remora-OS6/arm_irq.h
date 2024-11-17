#ifndef __ARM_IRQ_H
#define __ARM_IRQ_H

#include <stdint.h>

typedef unsigned long irqstatus_t;
#define __CORTEX_M                (7U)

 void irq_disable(void);
 void irq_enable(void);
irqstatus_t irq_save(void);
 void irq_restore(irqstatus_t flag);
 void irq_wait(void);
 void irq_poll(void);

#endif // irq.h
