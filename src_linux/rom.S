    la a1, 0x000000001000b0b0 # device tree follows the end of this
    csrr a0, mhartid
    li t0, 0x0000000010000000
    jr t0
