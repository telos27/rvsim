// clint.c: CLINT, UART emulation
#include <stdint.h>
#include <windows.h>
#include <conio.h>
#include <stdio.h>

#include "clint.h"


// CLINT I/O register states
static uint32_t timer_l = 0;	// part of CLINT, mtime in SiFive doc
static uint32_t timer_h = 0;
static uint32_t timer_match_l = 0;	// part of CLINT, mtimecmp in SiFive doc
static uint32_t timer_match_h = 0;	


uint32_t io_read(uint32_t addr, uint32_t *data)
{
	switch (addr) {
	case IO_CLINT_TIMERL: *data = timer_l; break;
	case IO_CLINT_TIMERH: *data = timer_h; break;
//	case IO_CLINT_TIMERMATCHL: *data = timer_match_l; break;
//	case IO_CLINT_TIMERMATCHH: *data = timer_match_h; break;
	// emulate UART behavior
	case IO_UART_DATA: *data = IsKBHit() ? ReadKBByte() : 0; break;
	case IO_UART_READY: *data = 0x60|IsKBHit(); break;
	default: break;
	}
	return 0;
}

uint32_t io_write(uint32_t addr, uint32_t* data)
{
	switch (addr) {
//	case IO_CLINT_TIMERL: timer_l = *data; break;
//	case IO_CLINT_TIMERH: timer_h = *data; break;
	case IO_CLINT_TIMERMATCHL: timer_match_l = *data; break;
	case IO_CLINT_TIMERMATCHH: timer_match_h = *data; break;
	// emulate UART behavior
	case IO_UART_DATA: printf("%c", *data); fflush(stdout); break ;
	case IO_DEBUG: debug_syscall(); break;	// debugging only
	default: break;
	}
	return 0;
}


// get current microsecond, Windows-specific, need future porting
uint64_t get_microseconds()
{
	static LARGE_INTEGER lpf;
	LARGE_INTEGER li;

	if (!lpf.QuadPart)
		QueryPerformanceFrequency(&lpf);

	QueryPerformanceCounter(&li);
	return ((uint64_t)li.QuadPart * 1000000LL) / (uint64_t)lpf.QuadPart;
}


// Windows-specific
static int IsKBHit()
{
	return _kbhit();
}

// Windows-specific
static int ReadKBByte()
{
	// This code is kind of tricky, but used to convert windows arrow keys
	// to VT100 arrow keys.
	static int is_escape_sequence = 0;
	int r;
	if (is_escape_sequence == 1)
	{
		is_escape_sequence++;
		return '[';
	}

	r = _getch();

if (is_escape_sequence)
	{
		is_escape_sequence = 0;
		switch (r)
		{
		case 'H': return 'A'; // Up
		case 'P': return 'B'; // Down
		case 'K': return 'D'; // Left
		case 'M': return 'C'; // Right
		case 'G': return 'H'; // Home
		case 'O': return 'F'; // End
		default: return r; // Unknown code.
		}
	}
	else
	{
		switch (r)
		{
		case 13: return 10; //cr->lf
		case 224: is_escape_sequence = 1; return 27; // Escape arrow keys
		default: return r;
		}
	}
}


static uint64_t last_time = 0;		// last time when we updated the timer, initialized in init_clint()

// CLINT: check to see if we should generate a timer interrupt
// return value: timer interrupt, 0 if no interrupt
// increment timer and also see if we've exceeded threshold
uint32_t run_clint()
{
	uint32_t gen_interrupt = 0xffffffff;

	uint64_t elapsed_time;
	uint32_t new_time;

	// update timer based on the current time
	elapsed_time = get_microseconds() - last_time;	
	last_time += elapsed_time;
	new_time = timer_l + elapsed_time;
	if (new_time < timer_l) timer_h++;	// timer_l overflowed
	timer_l = new_time;

	// compare timer and set interrupt pending info
	// MIP.MTIP is always updated: clear WFI & set MIP or clear MIP 
	uint32_t mip = read_CSR(CSR_MIP);
	if ((timer_match_h > 0 || timer_match_l > 0) && ((timer_h > timer_match_h) ||
		(timer_h == timer_match_h && timer_l >= timer_match_l))) {	// timer_match set and current time>=timer_match
		wfi = 0;
		write_CSR(CSR_MIP, mip | CSR_MIP_MTIP);		
	} else {
		write_CSR(CSR_MIP, mip & ~((uint32_t)CSR_MIP_MTIP));
	}

	uint32_t mstatus = read_CSR(CSR_MSTATUS);
	mip = read_CSR(CSR_MIP);
	uint32_t mie = read_CSR(CSR_MIE);
	// generate interrupt only if all three conditions are met:
	// MIP.MTIP , MIE.MTIE , MSTATUS.MIE
	if ((mip & CSR_MIP_MTIP) && (mie & CSR_MIE_MTIE) && (mstatus & CSR_MSTATUS_MIE)) {
		gen_interrupt = INTR_MTIMER;
	}

	return gen_interrupt;
}

// initialization routine
uint32_t init_clint()
{
	last_time = get_microseconds();
}
