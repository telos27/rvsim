	# RISC-V 32I
	# interrupt test
	# NOTE: depends on simulator's support for debugging syscall through I/O location 0x11200000

	.text

	li x8 , 0x11200000

	auipc t4 , 0
	addi t4 , t4 , 0x20
	csrrw t5 , mtvec , t4
	ecall

DONE:	
	li x5 , 10	
	sw x0 , 0x0(x8)
	li x5 , 1
	sw x0 , 0x0(x8)


ISR:
	csrrs t3 , mcause , zero
	csrrs t2 , mepc , zero
	addi t2 , t2 , 4
	csrrw t1 , mepc , t2
	mret
