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

	// Our stack lives at the top of the image's own footprint, so it needs no allocator and
	// cannot land in the window we have not accepted yet.
	adrp	x1, _stack_top
	add	x1, x1, :lo12:_stack_top
	mov	sp, x1

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
