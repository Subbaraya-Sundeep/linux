/*
 * This file contains driver for the Microsemi Smart Fusion2 Timer.
 *
 * Copyright (C) 2016 Sundeep Bhatta <sundeep.lkml@gmail.com>
 *
 * based on pit timer.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/clockchips.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/sched_clock.h>

#define MSF2_TIM1_LOADVAL	0x04
#define MSF2_TIM1_CTRL		0x0C
#define MSF2_TIM1_RIS		0x10
#define MSF2_TIM2_VAL		0x18
#define MSF2_TIM2_LOADVAL	0x1C
#define MSF2_TIM2_CTRL		0x24

#define TIMER_CTRL_ENBL		(1<<0)
#define TIMER_CTRL_ONESHOT	(1<<1)
#define TIMER_CTRL_INTR		(1<<2)
#define TIMER_RIS_ACK		(1<<0)
#define TIMER_RST_CLR		(1<<6)

/**
 * struct msf2_timer - This definition defines local timer structure
 *
 * @base_addr:	Base address of timer
 * @freq:	Timer input clock frequency
 * @clk:	Associated clock source
 * @clk_rate_change_nb	Notifier block for clock rate changes
 */
struct msf2_timer {
	void __iomem *base_addr;
	unsigned int irq;
	unsigned long freq;
	struct clock_event_device ce;
};

static void __iomem *msf2_sched_clock_val_reg;

/**
 * msf2_clock_event_interrupt - Clock event timer interrupt handler
 *
 * @irq:	IRQ number of the Timer
 * @dev_id:	void pointer to the msf2_timer instance
 *
 * returns: Always IRQ_HANDLED - success
 **/
static irqreturn_t msf2_clock_event_interrupt(int irq, void *dev_id)
{
	struct msf2_timer *timer = dev_id;

	/* Clear the interrupt */
	writel(TIMER_RIS_ACK, timer->base_addr + MSF2_TIM1_RIS);

	timer->ce.event_handler(&timer->ce);

	return IRQ_HANDLED;
}

static u64 notrace msf2_sched_clock_read(void)
{
	return ~readl(msf2_sched_clock_val_reg);
}

static int msf2_set_next_event(unsigned long cycles,
					struct clock_event_device *evt)
{
	struct msf2_timer *timer = container_of(evt, struct msf2_timer, ce);
	u32 ctrl_reg;

	writel(cycles, timer->base_addr + MSF2_TIM1_LOADVAL);

	ctrl_reg = readl(timer->base_addr + MSF2_TIM1_CTRL);
	ctrl_reg |= TIMER_CTRL_ENBL | TIMER_CTRL_INTR | TIMER_CTRL_ONESHOT;
	writel(ctrl_reg, timer->base_addr + MSF2_TIM1_CTRL);

	return 0;
}

static int msf2_shutdown(struct clock_event_device *evt)
{
	struct msf2_timer *timer = container_of(evt, struct msf2_timer, ce);
	u32 ctrl_reg;

	ctrl_reg = readl(timer->base_addr + MSF2_TIM1_CTRL);
	ctrl_reg &= ~TIMER_CTRL_ENBL;
	writel(ctrl_reg, timer->base_addr + MSF2_TIM1_CTRL);

	return 0;
}

static int msf2_set_periodic(struct clock_event_device *evt)
{
	struct msf2_timer *timer = container_of(evt, struct msf2_timer, ce);
	u32 ctrl_reg;
	unsigned long cycles = DIV_ROUND_CLOSEST(timer->freq, HZ);

	writel(cycles, timer->base_addr + MSF2_TIM1_LOADVAL);

	ctrl_reg = readl(timer->base_addr + MSF2_TIM1_CTRL);
	ctrl_reg |= TIMER_CTRL_ENBL | TIMER_CTRL_INTR;
	writel(ctrl_reg, timer->base_addr + MSF2_TIM1_CTRL);

	return 0;
}

static int msf2_resume(struct clock_event_device *evt)
{
	struct msf2_timer *timer = container_of(evt, struct msf2_timer,
					ce);
	u32 ctrl_reg;

	ctrl_reg = readl(timer->base_addr + MSF2_TIM1_CTRL);
	ctrl_reg |= TIMER_CTRL_ENBL;
	writel(ctrl_reg, timer->base_addr + MSF2_TIM1_CTRL);

	return 0;
}

static void __init msf2_setup_clocksource(struct msf2_timer *timer)
{
	writel(0xFFFFFFFF,  timer->base_addr + MSF2_TIM2_LOADVAL);
	writel(TIMER_CTRL_ENBL, timer->base_addr + MSF2_TIM2_CTRL);

	msf2_sched_clock_val_reg = timer->base_addr + MSF2_TIM2_VAL;
	sched_clock_register(msf2_sched_clock_read, 32, timer->freq);
	clocksource_mmio_init(timer->base_addr + MSF2_TIM2_VAL,
			"msf2_clocksource", timer->freq, 300, 32,
			clocksource_mmio_readl_down);
}

static void __init msf2_setup_clockevent(struct msf2_timer *timer)
{
	int err;

	timer->ce.name = "msf2_clockevent";
	timer->ce.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;
	timer->ce.set_next_event = msf2_set_next_event;
	timer->ce.set_state_shutdown = msf2_shutdown;
	timer->ce.set_state_periodic = msf2_set_periodic;
	timer->ce.set_state_oneshot = msf2_shutdown;
	timer->ce.tick_resume = msf2_resume;
	timer->ce.rating = 200;
	timer->ce.irq = timer->irq;
	timer->ce.cpumask = cpu_all_mask;

	err = request_irq(timer->irq, msf2_clock_event_interrupt,
			  IRQF_TIMER, timer->ce.name, timer);
	if (WARN_ON(err)) {
		kfree(timer);
		return;
	}

	clockevents_config_and_register(&timer->ce,
			timer->freq, 2, 0xffffffff);
}

/**
 * msf2_timer_init - Initialize the timer
 *
 * Initializes the timer hardware and register the clock source and clock event
 * timers with Linux kernal timer framework
 */
static void __init msf2_timer_init(struct device_node *timer_node)
{
	unsigned int irq;
	struct msf2_timer *timer;
	int ret;

	timer = kzalloc(sizeof(*timer), GFP_KERNEL);
	if (WARN_ON(!timer))
		return;

	timer->base_addr = of_iomap(timer_node, 0);
	if (!timer->base_addr) {
		pr_err("ERROR: invalid timer base address\n");
		BUG();
	}

	irq = irq_of_parse_and_map(timer_node, 0);
	if (irq <= 0) {
		pr_err("ERROR: invalid interrupt number\n");
		BUG();
	}

	timer->irq = irq;

	ret = of_property_read_u32(timer_node, "clock-frequency", &timer->freq);
	if (ret < 0) {
		pr_err("ERROR: invalid timer base address\n");
		BUG();
	}

	msf2_setup_clocksource(timer);
	msf2_setup_clockevent(timer);

	pr_info("%s at %p, irq=%d\n", timer_node->name, timer->base_addr, irq);
}

CLOCKSOURCE_OF_DECLARE(msf2, "ms,msf2-timer", msf2_timer_init);
