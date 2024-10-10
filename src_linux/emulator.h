
/**
 * This header file contains a riscv-emulator implemented
 * as a class. This provides an easy way to use the same
 * emulator code for testing the two processors.
 */

#define MEM_SIZE 28
#define NUM_HARTS 2

/* EMULATOR INCLUDE HEADER FILES */

#include <vector>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include "hart.h"

using namespace std;

class emulator
{
private:
vector<uint64_t> memory = vector<uint64_t>(1 << MEM_SIZE); // main memory
uint64_t mtimecmp[2];
uint32_t msip[2];
vector<hart> harts;

public:

  emulator():harts(NUM_HARTS,memory) // this Constructor will construct each harts
  {
    for(uint8_t i=0; i<NUM_HARTS; i++)
      harts[i].hart_init(memory,i);
  }

  void init(string image_name)
  {
    ifstream infile(image_name, ios::binary);
    printf("stepping\n");
    if (!infile.good())
    {
      exit(0);
    }

    // Get the file size by seeking to the end and then getting the position
    infile.seekg(0, ios::end);
    streampos fileSize = infile.tellg();
    infile.seekg(0, ios::beg);

    // Create a vector to store the binary data
    vector<unsigned long> byte_memory(fileSize);

    // Read the binary data into the vector
    infile.read(reinterpret_cast<char *>(byte_memory.data()), fileSize);

    infile.close();
    unsigned long pointer_end = (fileSize / 8) - 1;
    unsigned long long_jump = 0;
    for (const uint64_t data : byte_memory)
    {
      memory.at(long_jump) = (static_cast<unsigned long>(data));
      // printf("%lx\n", static_cast<unsigned long>(address));
      if ((long_jump++) >= pointer_end)
      {
        break;
      }
    }

    /// inittilize mem
  }

  void step()
  {
    for (auto &r : harts)
      r.hart_step(memory,mtimecmp,msip);

  }

  void set_interrupts()
  {
    for (auto &r : harts)
      r.hart_set_interrupts(mtimecmp,msip);
  }

  void show_registers()
  {
    for (auto &r : harts)
      r.show_state();
  }

};