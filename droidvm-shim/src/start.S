// Copyright 2026 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Entry. The resource manager starts this VM at the base of its lent region with x0 pointing at
// the device tree, MMU and caches off, interrupts masked, and nothing else worth trusting -- SP
// included, which is why the first thing here is to give ourselves one.
//
// Offset 0 must be a branch: crosvm patches the header that follows it, and the hypervisor's
// entry point is the base of the image either way.

.section .head, "ax"
.globl _start
_start:
	b	_entry				// +0x00
	.balign 8
	.globl shim_header
shim_header:				// +0x08, struct ShimHeader
	.quad	0x4d534856554d5644		// magic "DVMUVHSM"
	.word	1				// version
	.word	0				// flags
	.quad	0				// payload
	.quad	0				// handoff
	.quad	0				// dtb_max_size
	.quad	0, 0, 0				// reserved

.section .text
_entry:
	mov	x19, x0				// the device tree pointer, ours to keep

	// First sign of life, before anything can go wrong: one byte straight at the 8250 crosvm
	// puts at 0x3f8. With the MMU off this is a Device store that leaves the VM immediately,
	// so it needs no memory, no stack and no shared page to be readable on the host -- which
	// is exactly what makes it the right thing to trust when the shared page is in doubt.
	mov	x1, #0x3f8
	mov	w2, #'H'
	strb	w2, [x1]

	// Our stack lives at the top of the image's own footprint, so it needs no allocator and
	// cannot land in the window we have not accepted yet.
	adrp	x1, _stack_top
	add	x1, x1, :lo12:_stack_top
	mov	sp, x1

	// Exception vectors, before anything that could take one.
	//
	// Without them a fault at EL1 with the MMU off is an infinite exception loop: the vector
	// base is whatever the hypervisor left behind, the handler faults again, and the vcpu never
	// exits. The host CPU underneath it then stops answering IPIs, and the first
	// kick_all_cpus_sync to come along takes the phone down with a watchdog bite. That is a
	// long way to travel from a bad instruction in a 6 KiB shim, and it costs the reserve pool
	// and five minutes every time. With these, the same bad instruction prints one line.
	adrp	x1, _vectors
	add	x1, x1, :lo12:_vectors
	msr	vbar_el1, x1
	isb

	// Zero BSS: nothing has done it for us and the Rust side assumes it.
	adrp	x2, _bss_start
	add	x2, x2, :lo12:_bss_start
	adrp	x3, _bss_end
	add	x3, x3, :lo12:_bss_end
1:	cmp	x2, x3
	b.hs	2f
	str	xzr, [x2], #8
	b	1b
2:
	mov	x0, x19
	bl	shim_main			// returns the address to jump to
	mov	x1, x0
	mov	x0, x19				// Linux/EDK2 boot protocol: x0 = device tree

	// Everything we wrote went out with the MMU off, so it is already in memory rather than in
	// a dirty line -- but the payload we are about to enter was written by the HOST through a
	// cacheable mapping, and this CPU has never fetched from those addresses.
	dsb	sy
	ic	iallu
	dsb	sy
	isb

	mov	x2, xzr
	mov	x3, xzr
	mov	x4, xzr
	br	x1

// One entry per vector slot, each 0x80 bytes, the whole table 2 KiB aligned. Every one lands in
// the same place: there is nothing here that could sensibly handle an exception, and the only
// useful thing to do with one is say what it was.
.macro VECTOR_ENTRY index
	.balign 0x80
	mov	x0, #\index
	b	_exception
.endm

.section .text
.balign 0x800
.globl _vectors
_vectors:
	VECTOR_ENTRY 0			// current EL, SP0:  synchronous
	VECTOR_ENTRY 1			//                   IRQ
	VECTOR_ENTRY 2			//                   FIQ
	VECTOR_ENTRY 3			//                   SError
	VECTOR_ENTRY 4			// current EL, SPx:  synchronous
	VECTOR_ENTRY 5			//                   IRQ
	VECTOR_ENTRY 6			//                   FIQ
	VECTOR_ENTRY 7			//                   SError
	VECTOR_ENTRY 8			// lower EL, aarch64: synchronous
	VECTOR_ENTRY 9			//                   IRQ
	VECTOR_ENTRY 10			//                   FIQ
	VECTOR_ENTRY 11			//                   SError
	VECTOR_ENTRY 12			// lower EL, aarch32: synchronous
	VECTOR_ENTRY 13			//                   IRQ
	VECTOR_ENTRY 14			//                   FIQ
	VECTOR_ENTRY 15			//                   SError

_exception:
	// The stack may be the reason we are here, so stand up a known-good one of our own rather
	// than trusting sp. Nothing returns from this.
	adrp	x4, _stack_top
	add	x4, x4, :lo12:_stack_top
	mov	sp, x4
	mrs	x1, esr_el1
	mrs	x2, elr_el1
	mrs	x3, far_el1
	bl	shim_exception
	b	.
