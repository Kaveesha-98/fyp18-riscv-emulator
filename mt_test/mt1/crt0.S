  .text
  #.section .crt
  .global _start
  .type   _start, @function

_start:

  # Initialize global pointer
.option push
.option norelax
	la		gp,__global_pointer$
.option pop

	# Only let core zero Initialize everything while the others wait!
	# the only thing that we can count on right now is that the TEXT and DATA
	# are Initialized. Therefore we can make ALL the harts with ID !=0 wait
	# until a flag in the DATA section indicates that hart 0 is has finsihed
	# initializing everything.

	li		sp,0x0000000010010000	# sp address must be greter that DRAM base address and less than DRAM base address + 0x9000000
	csrrs	x10,mhartid,x0			# load my hart ID into x10
	bne		x10,x0,sync_all_harts

	# Only hart 0 will proceed into this code

	# Clear the bss segment

	la      a0,_edata				# a0 = start zeroing momory here
	la      a1,_end					# a1 = end zeroing here -1
	li 		a2,0

clear_bss:

	bgeu	a0,a1,finish_bss
	sb		a2,0(a0)
	addi	a0,a0,1
	beq		x0,x0,clear_bss

finish_bss:

	#call    __libc_init_array       # Run global initialization functions

	la		x10, hart_init_sync
	sw		x0,0(x10)				# store zero into hart_init_sync to relesase all the harts

sync_all_harts:

	# wait until the word at hart_init_sync is Zero

	la 		x10,hart_init_sync
	lw		x11,0(x10)
	bne		x11,x0,sync_all_harts

	# Each hart has to have a stack this code will assign a stack based on
	# the # hartid number is assumed to have been assigned as  0, 1, 2, ...
	# assigned sequentially.

	# Assume that each stack will be 4K in size.

	csrrs	x10,mhartid,x0			# load my hart ID into 0x10
	lui		x11,0x00001				# x11 = 0x00001000 = 4K


stack_loop:

	beq		x10,x0,finish_stack
	sub		x2,x2,x11				# x2 = sp = sp - 4K
	addi	x10,x10,-1
	beq		x0,x0,stack_loop

finish_stack:

	#initialize the argumnets to main() in case they are used

	li		a0,0					# a0 = argc = 0
	li		a1,0					# a1 = argv = NULL
	li		a2,0					# a2 = envp = NULL (this is a very common non-standard feature)

	call main

	# abort execution here

	ebreak			

	.size  _start, .-_start

	.data

hart_init_sync:
	.word 1							# hart zero will set this to 0 when the init logic has completed
