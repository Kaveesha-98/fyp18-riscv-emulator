/**
 * This header file contains a riscv-emulator implemented
 * as a class. This provides an easy way to use the same
 * emulator code for testing the two processors.
*/

/* EMULATOR INCLUDE HEADER FILES */
#include <unistd.h>
#include <cstdlib>
#include <signal.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <string>

// disables echo for terminal
void enable_raw_mode() {
  termios term;
  tcgetattr(0, &term);
  term.c_lflag &= ~(ICANON | ECHO); // Disable echo as well
  tcsetattr(0, TCSANOW, &term);
}

// return terminal back to original state before termination of the program
void disable_raw_mode() {
  termios term;
  tcgetattr(0, &term);
  term.c_lflag |= ICANON | ECHO;
  tcsetattr(0, TCSANOW, &term);
}

// detecting a key hit
int kbhit() {
  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);
  return byteswaiting > 0;
}

/**
 * Main emulator class, the following functions are defined.
 * 
 * @fn step() - Steps through one architectural change of pc
 * which includes executing one legal instruction, moving to
 * exception/interrupt handler after a synchronous/asynchronous
 * exception
 * @fn init() - Initializes the emulator, i.e. 
 *  1. Setting the pc to the first instruction to be executed
 *  2. Setting gprs and csrs
 *  3. Initializes the memory with the image of the program
 * @fn memory_read() - Performs a memory read at the given 
 * physical address. The read is a 8 byte read at a given
 * address.
 * @fn memory_write() - Performs a memory write at the given
 * physical address, at the specified size
 * @fn set_interrupts() - Looks for interrupts. If any are set,
 * then in the next step() the pc moves to the exception handler
 * @fn csr_set() - Performs settings CSRs such as mcycle
 * @fn fetch_instruction() - Performs a memory read for an 
 * instruction
 * 
 * The above functions may be overwritten when adopting the same
 * code for co-simulation and emulation with hardware simulation
 * of a RISC-V core
 * e.g.: To match peripheral accesses between the emulator and
 * simulator are synchrocous, memory_read() will change to provide
 * this functionality.
*/
class emulator {
  private:
  /**
   * Initiate all the variables needed to define the state of the
   * emulator.
   * e.g.: gprs, csrs, memory, etc...
  */
  public:
  /**
   * Initializes the emulator, i.e. 
   * 1. Setting the pc to the first instruction to be executed
   * 2. Setting gprs and csrs
   * 3. Initializes the memory with the image of the program
   * @param image_name filename of the kernel image to be loaded to 
   *  emulator memory
   * return 0 - to signal an error
  */
  int init(std::string image_name) {
    return 1;
  }

  /**
   * Perform a memory read for an instruction
   * @param address pc of the instruction
  */
  __uint32_t fetch_instruction(__uint64_t address) {
    return 0UL;
  }

  /**
   * Perform a memory read for a data fetch, always read 8 bytes.
   * Both for peripherals and main memory
   * @param address physical base address of the data to be accessed
  */
  __uint64_t memory_read(__uint64_t address) {
    return 0UL;
  }

  /**
   * Performs a memory write to both main memory and peripherals
   * @param address physical base address to be written
   * @param data data to be stores right justified
   * @param size number of bytes to overwrite
  */
  void memory_write(__uint64_t address, __uint64_t data, __uint8_t size) {

  }

  /**
   * Sets up interrupts to execute instructions next
   * i.e.: sets up *TIP, *SIP, *EIP
  */
  void set_interrupts() {

  }

  /**
   * Steps through one architectural change of pc
   * which includes executing one legal instruction, moving to
   * exception/interrupt handler after a synchronous/asynchronous
   * exception
  */
  void step() {

  }
};