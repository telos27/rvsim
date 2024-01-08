#pragma once
#include <stdint.h>

// memory operations funct3
#define MEM_BYTE 0x0
#define MEM_HALFWORD 0x1
#define MEM_WORD 0x2
#define MEM_UBYTE 0x4
#define MEM_UHALFWORD 0x5

// memory operation: match PAGEFAULT interrupt
#define MEM_INSTR 0 
#define MEM_READ 1
#define MEM_WRITE 3

extern int pa_mem_interface(uint32_t mem_mode, unsigned int addr, int size, unsigned int* data, uint32_t* interrupt);

// returns 22-bit PPN corresponding to 20-bit VPN
uint32_t vpn2ppn(uint32_t va , uint32_t access_mode, uint32_t *interrupt);
