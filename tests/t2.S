	# RISC-V 32I
	# test: test a few different instructions
	# NOTE: depends on simulator's support for debugging syscall through I/O location 0x11200000
	

xor x11 , x11 , x11
addi x5 , x0 , 10
li x8 , 0x11200000
li x9 , 0x80000000
sw x0 , 0x0(x8)
ori x11 , x11 , 62
sw x11 , 0x100(x9)
lw x12 , 0x100(x9)
sw x0 , 0x0(x8)
beq x11 , x12 , L1
ori x1 , x0 , 100
sw x0 , 0x0(x8)
L1:
addi x5 , x0 , 1
sw x0 , 0x0(x8)

