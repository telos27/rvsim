// csr.c: CSR-related handling
#include <stdint.h>

#include "csr.h"

// machine-mode CSR, only defined the ones we currently use
#define CSR_MSTATUS 0x300
#define CSR_MISA 0x301
#define CSR_MIE 0x304
#define CSR_MTVEC 0x305
#define CSR_MSCRATCH 0x340
#define CSR_MEPC 0x341
#define CSR_MCAUSE 0x342
#define CSR_MTVAL 0x343
#define CSR_MIP 0x344
#define CSR_MTINST 0x34A
#define CSR_MTVAL2 0x34B


// MEMIO addresses
// on real systems these would be actual devices
#define IO_CLINT_TIMERL 0x1100bff8
#define IO_CLINT_TIMERH 0x1100bffc
#define IO_CLINT_TIMERMATCHL 0x11004000
#define IO_CLINT_TIMERMATCHH 0x11004004
#define IO_UART_DATA 0x10000000
#define IO_UART_READY 0x10000005


// explicit init function?
static uint32_t CSRs[4096];
static uint32_t timer_l = 0;	// part of CLINT, mtime in SiFive doc
static uint32_t timer_h = 0;
static uint32_t timer_match_l = 0;	// part of CLINT mtimecmp in SiFive doc
static uint32_t timer_match_h = 0;	


uint32_t read_CSR(uint32_t CSR_no)
{
	return CSRs[CSR_no];
}

uint32_t write_CSR(uint32_t CSR_no, uint32_t value)
{
	CSRs[CSR_no] = value;
	return 1;
}

uint32_t io_read(uint32_t addr, uint32_t *data)
{
	switch (addr) {
	case IO_CLINT_TIMERL: *data = timer_l; break;
	case IO_CLINT_TIMERH: *data = timer_h; break;
	case IO_CLINT_TIMERMATCHL: *data = timer_match_l; break;
	case IO_CLINT_TIMERMATCHH: *data = timer_match_h; break;
	case IO_UART_DATA: // TODO: read one byte if there is key hit break ;
	case IO_UART_READY: // TODO: is key data available break ;
	default: break;
	}
	return 0;
}

uint32_t io_write(uint32_t addr, uint32_t* data)
{
	switch (addr) {
	case IO_CLINT_TIMERL: timer_l = *data; break;
	case IO_CLINT_TIMERH: timer_h = *data; break;
	case IO_CLINT_TIMERMATCHL: timer_match_l = *data ; break;
	case IO_CLINT_TIMERMATCHH: timer_match_h = *data; break;
	case IO_UART_DATA: // TODO: write data to screen break ;
	default: break;
	}
	return 0;
}


// CLINT: check to see if we should generate a timer interrupt
// increment timer and also see if we've exceeded threshold
void run_clint()
{

}