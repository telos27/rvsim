#include "mmu.h"
#include "clint.h"

#define PTE_LEVELS 2
#define VPN_SEG1 0xffc00
#define VPN_SEG2 0x3ff
#define PTE_V 0x1
#define PTE_R 0x2
#define PTE_W 0x4
#define PTE_X 0x8
#define PTE_U 0x10
#define PTE_G 0x20
#define PTE_A 0x40
#define PTE_D 0x80
#define PTE_PPN 0xffffc000

typedef uint32_t pte;


// TODO: check for a bunch of error conditions
// translate 20-bit vpn va to 22-bit ppn
// returns 0 if there is any kind of fault, which is stored in interrupt
uint32_t vpn2ppn(uint32_t vpn , uint32_t mem_access_mode , uint32_t *interrupt)
{
	uint32_t ppn = read_CSR(CSR_SATP) & CSR_SATP_PPN;
	uint32_t vpn_segment[] = { vpn & VPN_SEG1 , vpn & VPN_SEG2 };	// two segements corresponding to two-level page table

	// loop two levels for Sv32
	for (int i = 0; i < PTE_LEVELS; i++)
	{
		uint32_t pte;
		// TODO: update correct interrupt
		pa_mem_interface(MEM_READ, ppn << 12 | vpn_segment[i], MEM_WORD, &pte);	// physical address, no translation
		if (*interrupt != 0xffffffff) return 0;

		if ((pte & PTE_V) == 0 || ((pte & PTE_W) && !(pte & PTE_R))) {
			*interrupt = INT_INSTR_PAGEFAULT + mem_access_mode ;
		}

		if (pte & PTE_W || pte & PTE_X) { // leaf pte
			uint32_t result = pte & PTE_PPN;	// TODO: megapage handling
			// check U & SUM & MXR
			// check megapage alignment
			// handle A&D update
			if ((pte & PTE_A) == 0 || (mem_access_mode == MEM_WRITE && ((pte & PTE_D) == 0))) {
				*interrupt = INT_INSTR_PAGEFAULT + mem_access_mode;
				//or ACCESS if pte PMP/PMA
			} else {
				return result;
			}
		}
		else {
			ppn = pte & PTE_PPN ;		// next-level PTE
		}
	}
	// TODO: how to avoid two addr translations for a single AMO?
	*interrupt = mem_access_mode + INT_INSTR_PAGEFAULT;	//no leaf pte
	return 0;
}
