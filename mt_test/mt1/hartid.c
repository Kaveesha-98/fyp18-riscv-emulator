
#include "hartid.h"	// always include my own decls so compiler will tell me if I mess up!


uint64_t get_hartid()
{
	uint64_t my_hart_id;
	asm volatile("csrrs %0,mhartid,x10" : "=r"(my_hart_id) :);
	return my_hart_id;

}


