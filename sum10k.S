	# RISC-V 32I
	# sum 1 to 10K
	# NOTE: depends on simulator's support for debugging syscall through I/O location 0x11200000
	

	xor t3 , t3 , t3 	# sum
	li t4 , 0x1		# i
	li t5 , 10000		# loop limit

	li x8 , 0x11200000

LOOP:	
	bgt t4 , t5 , DONE
	add t3 , t3 , t4	# sum += i
	addi t4 ,t4 , 1		# i++ 
	j LOOP

DONE:
	li x5 , 10	
	sw x0 , 0x0(x8)
	li x5 , 1
	sw x0 , 0x0(x8)
