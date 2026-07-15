/*
 * Copyright (c) 2026 Andes Technology Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/riscv/irq.h>
#include "csr_irq.inc"

/* exports */
GTEXT(__soc_handle_exception)
GTEXT(rv_exception_handler)
/* imports */
GTEXT(_isr_wrapper)

/* Used to record that the exception was a trap or local interrupt */
GDATA(_trap_active)

/*
 * Each Andes Vectored PLIC vector table entry is 4 bytes for both RV32 and RV64.
 * On RV64, the upper 32 bits of the ISR address are taken from mtvec[63:32].
 *
 * The table base must be aligned to a 2^log2(2N+2) byte boundary, where N is
 * the number of PLIC interrupt sources. For example, if N is 1023, the minimum
 * alignment requirement is 4096 bytes.
 *
 * Vector table entries:
 * - mtvec[0] is used for exceptions and non-external local interrupts.
 * - mtvec[i] is used for PLIC external interrupt source i.
 */

__soc_handle_exception:
	/*
	 * Andes Vectored PLIC hardware automatically claims the pending
	 * interrupt and records the source ID in mcause.
	 *
	 * Check if _trap_active is set
	 */
	addi sp, sp, -16
	sr t0, 0x00(sp)
	sr t1, 0x08(sp)
	la t0, _trap_active
	lr t1, 0(t0)
	/* Clear _trap_active */
	sr x0, 0(t0)
	/* If _trap_active was set before, skip mcause handling */
	bnez t1, _soc_exception_done
	/*
	 * _trap_active was clear. We need to store the exception
	 * code in mcause as the interrupt source ID + CONFIG_2ND_LVL_ISR_TBL_OFFSET,
	 * so that the _isr_wrapper picks the correct offset in the SW isr table.
	 */
	csrr t0, mcause
	addi t0, t0, CONFIG_2ND_LVL_ISR_TBL_OFFSET
	/* Set the INTERRUPT bit in mcause */
	li t1, RISCV_MCAUSE_IRQ_BIT
	or t0, t0, t1
	/* Store to mcause */
	csrw mcause, t0
_soc_exception_done:
	lr t0, 0x00(sp)
	lr t1, 0x08(sp)
	addi sp, sp, 16
	ret

/*
 * This handler is installed at mtvec[0] for handling
 * exceptions and local interrupts, since they don't require special mcause
 * modification.
*/
rv_exception_handler:
	addi sp, sp, -16
	sr t0, 0x00(sp)
	sr t1, 0x08(sp)
	/* Set trap_active to true */
	la t0, _trap_active
	li t1, 1
	sr t1, 0(t0)
	lr t0, 0x00(sp)
	lr t1, 0x08(sp)
	addi sp, sp, 16
	/* Jump to _isr_wrapper */
	j _isr_wrapper
