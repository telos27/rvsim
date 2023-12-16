This is a modified form of the official test suite, from regymm/quasiSoC/software/tests.

Build the binary image firmware/firmware.bin with "make". Manually configure before make:
1. Cross-compiling toolchain path RISCV_PATH in Makefile, currently it is /opt/riscv32/bin
2. The address in firmware/linker.ld needs to match the starting address in the emulator, current it is 0x20000000
 

The fimrware/start.S/uart_putchar routine is based on simulator's simulated serial port. 

