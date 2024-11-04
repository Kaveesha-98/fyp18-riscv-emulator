/**
 * Run a rv-test given as the first command, return non-zero
 * value if test fails
 */

#include "emulator.h"

emulator emu;

#define TICK_LIMIT 10000 // How many instructions to execute

int main(int argc, char* argv[]) {
  // initaite registers and memory
  emu.init(argv[1]);
	printf("Running riscv-test image at %s \t result: ", argv[1]);

  for (uint32_t i = 0; i < TICK_LIMIT; i++) {
    // steps through one architectural change of pc
    emu.step();

    // sets up interrupts to move to exception handler
    // in next step()
    emu.set_interrupts();

		uint64_t ret = emu.memory_read(0x10001000);
		switch (ret) {
		case 0: // test not finished yet
			break;
		case 1: // test passed
			printf("passed \n");
			return 0;
		
		default: // test failed
			printf("failed at test: %03ld \n", ret);
			return 1;
			break;
		};
  }
	// timeout
	printf("failed due to timed out \n");
  return 2;
}
