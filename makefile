# Makefile for rvsim project

CC=gcc -g -Wall
TARGET=rv-emulator

all: $(TARGET)

sim: rv-emulator
	./rv-emulator tests/localbuild.bin tests/64mb.dtb

$(TARGET): rv-emulator.c clint.c mmu.c
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -f $(TARGET)

