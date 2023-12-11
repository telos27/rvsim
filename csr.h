#pragma once

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
#define CSR_MVENDORID 0xf11


// MEMIO addresses
// on real systems these would be actual devices
#define IO_CLINT_TIMERL 0x1100bff8
#define IO_CLINT_TIMERH 0x1100bffc
#define IO_CLINT_TIMERMATCHL 0x11004000
#define IO_CLINT_TIMERMATCHH 0x11004004
#define IO_UART_DATA 0x10000000
#define IO_UART_READY 0x10000005

// debugging use only, don't use in production code
#define IO_DEBUG 0x11200000

uint32_t read_CSR(uint32_t CSR_no);
uint32_t write_CSR(uint32_t CSR_no, uint32_t value);

uint32_t io_read(uint32_t addr, uint32_t* data);
uint32_t io_write(uint32_t addr, uint32_t* data);

uint32_t run_clint();