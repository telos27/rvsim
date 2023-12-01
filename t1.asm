xor x11 , x11 , x11
addi x5 , x0 , 10
ecall
ori x11 , x11 , 62
sw x11 , 0x100(x0)
lw x12 , 0x100(x0)
ecall
beq x11 , x12 , L1
ori x1 , x0 , 100
ecall
L1:
addi x5 , x0 , 1
ecall

