#include <stdio.h>


void set_rom()
{
  const int reset_vec_size = 8;

  int64_t start_pc = DRAM_BASE;

  uint32_t reset_vec[reset_vec_size] = {
    0x297,                                      // auipc  t0,0x0
    0x28593 + (reset_vec_size * 4 << 20),       // addi   a1, t0, &dtb
    0xf1402573,                                 // csrr   a0, mhartid
    0x0182b283u,                                // ld     t0,24(t0)
    0x28067,                                    // jr     t0
    0,
    (uint32_t) (start_pc & 0xffffffff),
    (uint32_t) (start_pc >> 32)
  };

  std::vector<char> rom((char*)reset_vec, (char*)reset_vec + sizeof(reset_vec));

  const int align = 0x1000;
  rom.resize((rom.size() + align - 1) / align * align);

}