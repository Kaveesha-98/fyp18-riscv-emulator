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

        la     ra,0x0000000000000078
    	li     sp,0x0000000010010000
    	sd     ra,8(sp)
    	ld     ra,8(sp)
    	ebreak
