// clint.c: CLINT, UART, PLIC, virtio emulation
#include <stdint.h>
#include <stdio.h>
#include <assert.h>

#if defined(WINDOWS) || defined(WIN32) || defined(_WIN32)
#include <windows.h>
#include <conio.h>
#else
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#endif


#include "clint.h"

#include "mmu.h"


uint32_t vio_interrupt_pending();


// CLINT I/O register states
static uint32_t timer_l = 0;	// part of CLINT, mtime in SiFive doc
static uint32_t timer_h = 0;
static uint32_t timer_match_l = 0;	// part of CLINT, mtimecmp in SiFive doc
static uint32_t timer_match_h = 0;	

// UART state
static uint32_t uart_interrupt = 0;
static uint32_t uart_interrupt_pending;

// virtio state
static uint32_t virtio_interrupt_pending;

// PLIC state
static uint32_t plic_priority[128];
static uint32_t plic_pending[4];
static uint32_t plic_enable[4];
static uint32_t plic_threshold, plic_claim;


uint32_t plic_read(uint32_t addr, uint32_t* data)
{
	if (addr >= PLIC_PRIORITY_START && addr < PLIC_PRIORITY_END) {
		*data = plic_priority[addr - PLIC_PRIORITY_START];
	}
	else if (addr >= PLIC_PENDING_START && addr < PLIC_PENDING_END) {
		*data = plic_pending[addr - PLIC_PENDING_START];
	} if (addr >= PLIC_ENABLE_START && addr < PLIC_ENABLE_END) {
		*data = plic_enable[addr - PLIC_ENABLE_START];
	}
	else if (addr == PLIC_THRESHOLD) {
		*data = plic_threshold;
	}
	else if (addr == PLIC_CLAIM) {
		*data = plic_claim;
	}
	else {
		assert(0);
	}
}


uint32_t plic_write(uint32_t addr, uint32_t* data)
{
	if (addr >= PLIC_PRIORITY_START && addr < PLIC_PRIORITY_END) {
		plic_priority[addr - PLIC_PRIORITY_START] = *data;
	}
	else if (addr >= PLIC_PENDING_START && addr < PLIC_PENDING_END) {
		plic_pending[addr - PLIC_PENDING_START] = *data;
	} if (addr >= PLIC_ENABLE_START && addr < PLIC_ENABLE_END) {
		plic_enable[addr - PLIC_ENABLE_START] = *data ;
	}
	else if (addr == PLIC_THRESHOLD) {
		plic_threshold = *data;
	}
	else if (addr == PLIC_CLAIM) {
		// clear corresponding pending bit
		uint32_t* p = plic_pending[*data >> 5];
		*p &= ~(1 << (*data & 0x1f));

		plic_claim = 0 ;		// TODO: should be next pending interrupt? current way only works if there is single pending interrupt
	}
	else {
		assert(0);
	}

}


uint32_t run_plic()
{
	// rvemu seems to assume a single priority pending, it should calculate according to priority and pending
	if (uart_interrupt_pending) {
		plic_pending[0] |= 1 << 10;
		plic_claim = 10;
	}
	else if (vio_interrupt_pending()) {
		vio_disk_access();
		plic_pending[0] |= 1 << 1;
		plic_claim = 1;
	}
	return 0;
}


// virtio implementation here is written to fit with xv6 and legacy mode in qemu?
// biggest difference with recent virtio spec is single pfn vs. three descriptor addresses
#define VIO_QUEUE_SIZE 8
#define SECTOR_SIZE 512

#define VRING_DESC_F_NEXT  1 // chained with another descriptor
#define VRING_DESC_F_WRITE 2 // device writes (vs read)

static uint32_t vio_page_size;
static uint32_t vio_queue_num;
static uint32_t vio_pfn;
static uint32_t vio_queue_align;
static uint32_t vio_status;
static uint32_t vio_queue_notify = UINT32_MAX;
static uint32_t vio_used_idx;	// matches virtq.used.idx
static uint8_t* vio_disk;	// disk space, allocated in init

uint32_t init_vio()
{
	// empty right now
	vio_disk = malloc(64 * 1024 * 1024);	// 64M disk
}

uint32_t vio_read(uint32_t addr, uint32_t* data)
{
	uint32_t offset = addr - VIRTIO_START;
	switch (offset) {
	case VIRTIO_MAGIC_VALUE: *data = 0x74726976; break;
	case VIRTIO_VERSION: *data = 0x1; break;
	case VIRTIO_DEVICE_ID: *data = 0x2; break;
	case VIRTIO_VENDOR_ID: *data = 0x554d4551; break;
	case VIRTIO_DEVICE_FEATURES: *data = 0; break;	// appears unused
	case VIRTIO_QUEUE_NUM_MAX: *data = VIO_QUEUE_SIZE; break;
	default: assert(0);		// unsupported I/O register
	}
}


uint32_t vio_write(uint32_t addr, uint32_t* data)
{
	uint32_t offset = addr - VIRTIO_START;
	switch (offset) {
	case VIRTIO_STATUS: {
		vio_status = *data;	// vs. rvemu; I don't think needed for xv6
		break;
	}
	case VIRTIO_DRIVER_FEATURES: break;	// appears unused
	case VIRTIO_GUEST_PAGE_SIZE: vio_page_size = *data; break;
	case VIRTIO_QUEUE_SEL: assert(*data == 0); break;	// only single queue
	case VIRTIO_QUEUE_NUM: vio_queue_num = *data; break;
	case VIRTIO_QUEUE_ALIGN: vio_queue_align = *data; break;
	case VIRTIO_QUEUE_PFN: vio_pfn = *data; break;
	case VIRTIO_QUEUE_NOTIFY: vio_queue_notify = *data; break;	
	default: assert(0);		// unsupported I/O register
	}
}

// If there has been a notify, generate an interrupt and process the request
uint32_t vio_interrupt_pending()
{
	if (vio_queue_notify != UINT_MAX) {
		vio_queue_notify = UINT_MAX;
		return TRUE;
	}
	else {
		return FALSE;
	}
}

vio_disk_access()
{
	uint32_t interrupt;
	// read 3 descriptors from virtual queue

	// address of virt queue
	uint32_t virtq = vio_pfn*vio_page_size ;

	// address of avail 
	uint32_t avail = virtq + 16 * vio_queue_num;	// 16 is descriptor size
	uint32_t avail_idx;
	pa_mem_interface(MEM_READ, avail + 2, MEM_HALFWORD, &avail_idx, &interrupt);	// read avail.idx

	uint32_t head_index;
	pa_mem_interface(MEM_READ , avail+4 + avail_idx*2 , MEM_HALFWORD , &head_index, &interrupt);

	uint32_t desc0_addr;
	pa_mem_interface(MEM_READ, virtq + 16 * head_index, MEM_WORD, &desc0_addr, &interrupt);

	uint32_t sector;
	pa_mem_interface(MEM_READ, desc0_addr + 8, MEM_WORD, &sector, &interrupt);		// LSB

	uint32_t desc0_next;	// index for desc1
	pa_mem_interface(MEM_READ, virtq + 16 * head_index + 14, MEM_HALFWORD, &desc0_next, &interrupt);

	uint32_t desc1_addr;
	pa_mem_interface(MEM_READ, virtq + 16 * desc0_next , MEM_HALFWORD, &desc1_addr, &interrupt);

	uint32_t desc1_len;
	pa_mem_interface(MEM_READ, virtq + 16 * desc0_next + 8, MEM_HALFWORD, &desc1_len, &interrupt);

	uint32_t desc1_flags;
	pa_mem_interface(MEM_READ, virtq + 16 * desc0_next + 12, MEM_HALFWORD, &desc1_flags, &interrupt);

	uint32_t desc1_next;
	pa_mem_interface(MEM_READ, virtq + 16 * desc0_next + 14, MEM_HALFWORD, &desc1_next, &interrupt);

	uint32_t data;	// only 1 byte used

	if (desc1_flags & VRING_DESC_F_WRITE) {
		for (int i = 0; i < desc1_len; i++) {
			pa_mem_interface(MEM_READ, desc1_addr + i, MEM_BYTE , &data, &interrupt);
			vio_disk[sector * SECTOR_SIZE + i] = data;
		}
	}
	else {
		for (int i = 0; i < desc1_len; i++) {
			data = vio_disk[sector * SECTOR_SIZE + i];
			pa_mem_interface(MEM_WRITE, desc1_addr + i, MEM_BYTE, &data, &interrupt);
		}
	}

	// set desc2's block to zero to mean completion
	uint32_t desc2_addr;
	pa_mem_interface(MEM_READ, virtq + 16 * desc1_next, MEM_WORD, &desc2_addr, &interrupt);

	data = 0;
	pa_mem_interface(MEM_WRITE, desc2_addr, MEM_BYTE, &data, &interrupt);

	// update used
	// address of used
	uint32_t used = virtq + 4096;	// per xv6-rv32

	// update used.idx; 
	pa_mem_interface(MEM_WRITE, used + 4 + 8 * vio_used_idx, MEM_WORD, head_index, &interrupt);

	// update used.idx; vio_used_idx is same as used.idx
	vio_used_idx = (vio_used_idx + 1) % VIO_QUEUE_SIZE;
	pa_mem_interface(MEM_WRITE, used + 2, MEM_HALFWORD, &vio_used_idx, &interrupt);
}



uint32_t io_read(uint32_t addr, uint32_t *data)
{
	if (addr >= IO_PLIC_START && addr < IO_PLIC_END) {
		return plic_read(addr, data);
	}
	switch (addr) {
		case IO_CLINT_TIMERL: *data = timer_l; break;
		case IO_CLINT_TIMERH: *data = timer_h; break;
	//	case IO_CLINT_TIMERMATCHL: *data = timer_match_l; break;
	//	case IO_CLINT_TIMERMATCHH: *data = timer_match_h; break;
		// emulate UART behavior
		case IO_UART_DATA: *data = IsKBHit() ? ReadKBByte() : 0; break;
		case IO_UART_INTRENABLE: *data = uart_interrupt; break;  // should not be readable, but Linux seems to read it
		case IO_UART_INTRSTATUS: *data = 0; break;	// used by Linux driver?
		case IO_UART_LINECTRL: *data = 0; break;	// used by Linux driver?
		case IO_UART_MODEMCTRL: *data = 0; break;	// seems to be used by Linux driver
		case IO_UART_MODEMSTATUS: *data = 0; break;	// seems to be used by Linux driver
		case IO_UART_READY: *data = 0x60|IsKBHit(); break;
		default: assert(0);break;
	}
	return 0;
}

// TODO: special mode for UART baud latch?
uint32_t io_write(uint32_t addr, uint32_t* data)
{
	if (addr >= IO_PLIC_START && addr < IO_PLIC_END) {
		return plic_write(addr, data);
	}
	switch (addr) {
	//	case IO_CLINT_TIMERL: timer_l = *data; break;
	//	case IO_CLINT_TIMERH: timer_h = *data; break;
		case IO_CLINT_TIMERMATCHL: timer_match_l = *data; break;
		case IO_CLINT_TIMERMATCHH: timer_match_h = *data; break;
		// emulate UART behavior; LCR and FCR write should be ok
		case IO_UART_DATA: printf("%c", *data); fflush(stdout); break ;
		case IO_UART_INTRENABLE: uart_interrupt = *data; break;
		case IO_UART_LINECTRL: break;
		case IO_UART_FIFOCTRL: break;
		case IO_UART_MODEMCTRL: break;	// seems to be used by Linux driver
		case IO_DEBUG: debug_syscall(); break;	// debugging only
		default: assert(0);break;
	}
	return 0;
}

#if defined(WINDOWS) || defined(WIN32) || defined(_WIN32)

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

#else

uint64_t GetTimeMicroseconds()
{
	struct timeval tv;
	gettimeofday(&tv, 0);
	return tv.tv_usec + ((uint64_t)(tv.tv_sec)) * 1000000LL;
}

static int is_eofd;

static int ReadKBByte()
{
	if (is_eofd) return 0xffffffff;
	char rxchar = 0;
	int rread = read(fileno(stdin), (char*)&rxchar, 1);

	if (rread > 0) // Tricky: getchar can't be used with arrow keys.
		return rxchar;
	else
		return -1;
}

static int IsKBHit()
{
	if (is_eofd) return -1;
	int byteswaiting;
	ioctl(0, FIONREAD, &byteswaiting);
	if (!byteswaiting && write(fileno(stdin), 0, 0) != 0) { is_eofd = 1; return -1; } // Is end-of-file for 
	return !!byteswaiting;
}

#endif

static uint64_t last_time = 0;		// last time when we updated the timer, initialized in init_clint()

// CLINT: check to see if we should generate a timer interrupt
// return value: timer interrupt, 0 if no interrupt
// increment timer and also see if we've exceeded threshold
uint32_t run_clint()
{
	uint32_t gen_interrupt = 0xffffffff;

	uint64_t elapsed_time;
	uint32_t new_time;

	uint32_t mip, mie, mstatus;
	// check for external interrupt
	run_plic();

	mstatus = read_CSR(CSR_MSTATUS);
	mip = read_CSR(CSR_MIP);
	mie = read_CSR(CSR_MIE);
	if (plic_claim!=0 && (mip & CSR_MIP_MEIP) && (mie & CSR_MIE_MEIE) && (mstatus & CSR_MSTATUS_MIE)) {
		gen_interrupt = INTR_MEXTERNAL;
		return gen_interrupt;
	}

	// update timer based on the current time
	elapsed_time = get_microseconds() - last_time;	
	last_time += elapsed_time;
	new_time = timer_l + elapsed_time;
	if (new_time < timer_l) timer_h++;	// timer_l overflowed
	timer_l = new_time;

	// compare timer and set interrupt pending info
	// MIP.MTIP is always updated: clear WFI & set MIP or clear MIP 
	mip = read_CSR(CSR_MIP);
	if ((timer_match_h > 0 || timer_match_l > 0) && ((timer_h > timer_match_h) ||
		(timer_h == timer_match_h && timer_l >= timer_match_l))) {	// timer_match set and current time>=timer_match
		wfi = 0;
		write_CSR(CSR_MIP, mip | CSR_MIP_MTIP);		
	} else {
		write_CSR(CSR_MIP, mip & ~((uint32_t)CSR_MIP_MTIP));
	}

	mstatus = read_CSR(CSR_MSTATUS);
	mip = read_CSR(CSR_MIP);
	mie = read_CSR(CSR_MIE);
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

	init_vio();
}
