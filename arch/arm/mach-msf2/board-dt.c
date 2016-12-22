#include <linux/kernel.h>
#include <asm/v7m.h>
#include <asm/mach/arch.h>

static const char *const msf2_compat[] __initconst = {
	"ms,msf2",
	NULL
};

DT_MACHINE_START(SF2DT, "Microsemi SF2 (Device Tree Support)")
	.dt_compat = msf2_compat,
	.restart = armv7m_restart,
MACHINE_END
