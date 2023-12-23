#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h> //for exit()

#include "clint.h"
#include "mmu.h"


// initial instruction address upon startup
#define INITIAL_PC 0x80000000

// instruction decoding： bitfields of an instruction; could use an array
#define OPCODE_MASK 0x7f    // bits [6:0]
#define OPCODE_SHIFT 0
#define FUNCT7_MASK 0xfe000000  // bits [31:25]
#define FUNCT7_SHIFT 25
#define FUNCT3_MASK  0x7000   // bits [14:12]
#define FUNCT3_SHIFT 12
#define RD_MASK 0xf80       // bits [11:7]
#define RD_SHIFT 7
#define RS1_MASK 0xf8000    // bits [19:15]
#define RS1_SHIFT 15
#define RS2_MASK 0x1f00000   // bits [24:20]
#define RS2_SHIFT 20
#define IMM20_MASK 0xfffff000   // bits [31:12]
#define IMM20_SHIFT 12
#define IMM5_MASK RD_MASK
#define IMM5_SHIFT RD_SHIFT
#define IMM7_MASK FUNCT7_MASK
#define IMM7_SHIFT FUNCT7_SHIFT
#define IMM12_MASK 0xfff00000   // bits [31:20]
#define IMM12_SHIFT 20

// opcodes: RV32IMA_Zicsr_Zifenci, mret, wfi
#define OP_ADD 0b0110011
#define OP_ADDI 0b0010011
#define OP_LB 0b0000011
#define OP_SB 0b0100011
#define OP_BEQ 0b1100011
#define OP_JAL 0b1101111
#define OP_JALR 0b1100111
#define OP_LUI 0b0110111
#define OP_AUIPC 0b0010111
#define OP_ECALL 0b1110011
#define OP_FENCEI 0b0001111
#define OP_A 0b0101111

// R/I instruction funct3
#define ALU_ADD 0x0     // SUB funct7=0x20
#define ALU_XOR 0x4
#define ALU_OR 0x6
#define ALU_AND 0x7
#define ALU_SLL 0x1
#define ALU_SRL 0x5     // SRA/SRAI funct7=0x20
#define ALU_SLT 0x2
#define ALU_SLTU 0x3

// RV-M funct3
#define MUL 0x0
#define MULH 0x1
#define MULHSU 0x2
#define MULHU 0x3
#define DIV 0x4
#define DIVU 0x5
#define REM 0x6
#define REMU 0x7

// funct7: SUB & SRA
#define NORMAL 0x0
#define SUB 0x20
#define SRA 0x20

// branch funct3
#define BRANCH_EQ 0x0
#define BRANCH_NE 0x1
#define BRANCH_LT 0x4
#define BRANCH_GE 0x5
#define BRANCH_LTU 0x6
#define BRANCH_GEU 0x7

// ecall funct3
#define SYSTEM_ECALL 0x0
#define SYSTEM_CSRRW 0x1
#define SYSTEM_CSRRS 0x2
#define SYSTEM_CSRRC 0x3
#define SYSTEM_CSRRWI 0x5
#define SYSTEM_CSRRSI 0x6
#define SYSTEM_CSRRCI 0x7

// SYSTEM_ECALL imm12
#define ECALL_ECALL 0x0
#define ECALL_EBREAK 0x1
#define ECALL_MRET 0x302
#define ECALL_SRET 0x102
#define ECALL_WFI 0x105

// AMO funct7[6:4]
#define AMO_ADD 0x0
#define AMO_XOR 0x1
#define AMO_AND 0x3
#define AMO_OR 0x2
#define AMO_MIN 0x4
#define AMO_MAX 0x5
#define AMO_MINU 0x6
#define AMO_MAXU 0x7

// AMO funct7[3:2] for ADD
#define AMO_ADD_ADD 0x0
#define AMO_ADD_SWAP 0x1
#define AMO_ADD_LR 0x2
#define AMO_ADD_SC 0x3

// privilege levles
#define MODE_M 0x3
#define MODE_S 0x1
#define MODE_U 0x0


// usable memory size: 64M
#define MEMSIZE 64*1024*1024  

// machine code file name, if unspecificed
#define DEFAULT_FILE "asm.o"

#define BOOL int8_t
#define TRUE 1
#define FALSE 0 
#define REGISTER uint32_t   // enough to hold the content of a single register


// external memory
static uint8_t mem[MEMSIZE];   // main memory

// MEMIO range: currently CLINT & UART, and debug syscall
#define MEMIO_START 0x10000000
#define MEMIO_END 0x12000000    // exclusive

#define NO_CSRS 4096

// CPU internal state
static REGISTER regs[32];
static uint32_t CSRs[NO_CSRS];  // explicit init?
static unsigned int pc ;       // 32-bit PC
static unsigned int mode;      // privilege mode: currently M only
static unsigned int reservation;   // address for lr/sc ; top 29 bits
unsigned int wfi = 0;    // WFI flag 
unsigned int no_cycles; // execution cycles; currently always 1 cycle/instruction

static unsigned int trace = 0;  // trace every instruction
static unsigned int interrupt;  // interrupt type
static uint32_t dtb_offset;    // offset to DTB

// read register
// can optimize by always 0 in x0
REGISTER read_reg(int reg_no)
{
    return (reg_no == 0) ? 0 : regs[reg_no];
}

int write_reg(int reg_no, REGISTER data)
{
        if (reg_no != 0) {
            regs[reg_no] = data;
        }
        return reg_no;  // not used yet
}


uint32_t read_CSR(uint32_t CSR_no)
{
    if (CSR_no == CSR_MISA)
        return 0x40401101;
    else if (CSR_no == CSR_MVENDORID)
        return 0xff0fff0f;
    else if (CSR_no == CSR_CYCLE)
        return no_cycles;
    else if (CSR_no & 0xc00) {
        //printf("read_CSR: 0x%x\n", CSR_no);
    }
    return CSRs[CSR_no];
}

uint32_t write_CSR(uint32_t CSR_no, uint32_t value)
{
    CSRs[CSR_no] = value;
    return 1;
}



// physical address memory operation, no sign extension done here
// memory address start at INITIAL_PC, no memory storage below that address
// can improve perf by handling multiple bytes at once
int pa_mem_interface(uint32_t mem_mode, unsigned int addr, int size, unsigned int* data)
{
    assert(addr >= INITIAL_PC);
    // TODO: PMP & PMA checks 
    addr -= INITIAL_PC;
    assert(size == MEM_BYTE || size == MEM_HALFWORD || size == MEM_WORD || size==MEM_UBYTE || size==MEM_UHALFWORD);
    if (mem_mode == MEM_WRITE) {
        switch (size) {
        case MEM_BYTE:
            mem[addr] = (*data) & 0xff;
            break;
        case MEM_HALFWORD:
            mem[addr] = (*data) & 0xff;
            mem[addr + 1] = ((*data) & 0xff00) >> 8;
            break;
        case MEM_WORD:
            mem[addr] = (*data) & 0xff;
            mem[addr + 1] = ((*data) & 0xff00) >> 8;
            mem[addr + 2] = ((*data) & 0xff0000) >> 16;
            mem[addr + 3] = ((*data) & 0xff000000) >> 24;
            break;
        default: interrupt=INT_ILLEGAL_INSTR ; // unspported size
        }

    } else { // both instruction and data read
        switch (size) {
        case MEM_BYTE: case MEM_UBYTE:
            // NOTE: no sign extension
            *data = mem[addr];
            break;
        case MEM_HALFWORD: case MEM_UHALFWORD:
            // NOTE: no sign extension
            *data = mem[addr] | ((unsigned int)mem[addr + 1]) << 8 ;
            break;
        case MEM_WORD:
            *data = mem[addr] | ((unsigned int)mem[addr + 1]) << 8 |
                ((unsigned int)mem[addr + 2]) << 16 | ((unsigned int)mem[addr + 3]) << 24;
            break;
        default:
            interrupt =INT_ILLEGAL_INSTR ; // unsupported size
        }

    }
    return TRUE;
}


// read/write memory, instruction semantics, which includes sign extension in some cases
// include memory-mapped I/O in specificed address range, which goes through MMU as well
// NOTE: little-endian
int rw_memory(uint32_t mem_mode, uint32_t addr, int sub3, unsigned int* data)
{
    assert(sub3 == MEM_BYTE || sub3 == MEM_HALFWORD || sub3 == MEM_WORD || sub3 == MEM_UBYTE || sub3 == MEM_UHALFWORD);
    // address translation here ;
    uint32_t satp = read_CSR(CSR_SATP);

    if (mode!=MODE_M && ((satp & CSR_SATP_MODE) == 1)) {
        addr = (vpn2ppn(addr>>12 , mem_mode , &interrupt) << 12) | (addr & 0xfff);  // 34 bit pa, currently we only simulate 32-bit
        if (interrupt!=0xffffffff) return -1;
    }
    if (mem_mode==MEM_WRITE) {
        if (addr >= MEMIO_START && addr < MEMIO_END) {
            return io_write(addr, data);
        }
        else {
            return pa_mem_interface(mem_mode, addr, sub3, data);
        }
    } else {    // both instruction and data read
        uint32_t read_data;
        int result;
        if (addr >= MEMIO_START && addr < MEMIO_END) {
            result = io_read(addr, &read_data) ;  // TODO: what do we do about non-word-sized I/O？
        }
        else {
            result = pa_mem_interface(mem_mode, addr, sub3, &read_data);
        }
        switch (sub3) {
            case MEM_BYTE: 
                // NOTE: sign extension
                *data = (int32_t)((int8_t)read_data) ; 
                break ;
            case MEM_HALFWORD: 
                // NOTE: sign extension
                *data = (int32_t) ((int16_t)read_data)  ;
                break;
            case MEM_WORD: 
            case MEM_UBYTE:
            case MEM_UHALFWORD:
                *data = read_data ;
                break ;
            default: 
                interrupt = INT_ILLEGAL_INSTR; // undefined sub3
        }
        return result;

    }
}

// NOTE: shift amount only uses 5-bits
int reg_op(int rd , int rs1 , int rs2 , int sub3 , int sub7)
{
    if ((sub7 & 1) != 1) {
        switch (sub3) {
        case ALU_ADD:
            if (sub7 == NORMAL) {
                write_reg(rd, read_reg(rs1) + read_reg(rs2));
            }
            else if (sub7 == SUB) {
                write_reg(rd, read_reg(rs1) - read_reg(rs2));
            }
            else {
                interrupt = INT_ILLEGAL_INSTR;  // nonexistent sub7
            }
            break;
        case ALU_XOR:
            write_reg(rd, read_reg(rs1) ^ read_reg(rs2));
            break;
        case ALU_OR:
            write_reg(rd, read_reg(rs1) | read_reg(rs2));
            break;
        case ALU_AND:
            write_reg(rd, read_reg(rs1) & read_reg(rs2));
            break;
        case ALU_SLL:
            write_reg(rd, read_reg(rs1) << (read_reg(rs2) & 0x1f));
            break;
        case ALU_SRL:
            if (sub7 == NORMAL) {
                write_reg(rd, read_reg(rs1) >> (read_reg(rs2) & 0x1f));
            }
            else if (sub7 == SRA) {
                // signed right shift
                write_reg(rd, ((int32_t)read_reg(rs1)) >> (read_reg(rs2) & 0x1f));
            }
            else {
                interrupt = INT_ILLEGAL_INSTR;  // nonexistent sub7
            }
            break;
        case ALU_SLT:
            write_reg(rd, ((int32_t)read_reg(rs1) < (int32_t)read_reg(rs2)) ? 1 : 0);
            break;
        case ALU_SLTU:
            write_reg(rd, (read_reg(rs1) < read_reg(rs2)) ? 1 : 0);
            break ;
        default: interrupt = INT_ILLEGAL_INSTR ; // unknown sub3
        }
    } else { // MULDIV
        // NOTE: signedness, special cases for division/remainder of certain numbers
        uint32_t n1 = read_reg(rs1);
        uint32_t n2 = read_reg(rs2);
        uint32_t result = 0;
        switch (sub3) {
        case MUL: result = n1 * n2; break;
        case MULH: result = (((int64_t)(int32_t)n1) * ((int64_t)(int32_t)n2)) >> 32; break;
        case MULHSU: result = (((int64_t)(int32_t)n1) * ((uint64_t)n2)) >> 32; break;
        case MULHU:result = (((uint64_t)n1) * ((uint64_t)n2)) >> 32; break;
        case DIV: if (n2 == 0) result = -1; else
            result = ((((int32_t)n1) == INT32_MIN) && ((int32_t)n2) == -1) ? n1 : (((int32_t)n1) / (int32_t)n2); break;
        case DIVU: result = (n2 == 0) ? 0xffffffff : (n1 / n2); break;
        case REM: if (n2 == 0) result = n1; else
            result = ((((int32_t)n1) == INT32_MIN) && (((int32_t)n2) == -1)) ? 0 : 
                (uint32_t)((int32_t)n1 % (int32_t)n2); 
            break;
        case REMU: result = (n2 == 0) ? n1 : (n1 % n2); break;
        default: interrupt = INT_ILLEGAL_INSTR; break;  // unknown sub3
        }
        write_reg(rd, result);
    }
    return TRUE;
}

// NOTE: imm is already sign-extended; shift amount only uses 5 bits
int imm_op(int rd , int rs1 , int sub3 , int sub7 , unsigned int imm)
{
    switch (sub3) {
    case ALU_ADD: write_reg(rd, read_reg(rs1) + (int32_t) imm); break;
    case ALU_XOR: write_reg(rd, read_reg(rs1) ^ imm); break;
    case ALU_OR: write_reg(rd, read_reg(rs1) | imm); break;
    case ALU_AND: write_reg(rd, read_reg(rs1) & imm); break;
    case ALU_SLL: write_reg(rd, read_reg(rs1) << (imm & 0x1f)); break;
    case ALU_SRL:
        if (sub7 == NORMAL) {
            write_reg(rd, read_reg(rs1) >> (imm & 0x1f));
        }
        else if (sub7 == SRA) {
            // signed right shift
            write_reg(rd, ((int32_t)read_reg(rs1)) >> (imm & 0x1f));
        }
        else {
            interrupt = INT_ILLEGAL_INSTR; // nonexistent sub7
        }
        break;
    case ALU_SLT:
        write_reg(rd, ((int32_t)read_reg(rs1) < (int32_t)imm) ? 1 : 0);
        break;
    case ALU_SLTU:
        write_reg(rd, (read_reg(rs1) < imm) ? 1 : 0);
        break;
    default: interrupt = INT_ILLEGAL_INSTR; // nonexistent sub3
    }
    return TRUE;
}

// return a sign-extended version of a number with no_bits
int sign_extend(uint32_t n , int no_bits)
{
    // TODO: is this portable?
    return (((signed int)n) << (32 - no_bits)) >> (32 - no_bits);
}

// return next PC, -1 if we don't branch
int branch_op(int rs1 , int rs2 , int sub3 , unsigned int imm5 , unsigned int imm7)
{
    BOOL do_branch = FALSE ;

    switch (sub3) {
    case BRANCH_EQ: do_branch = read_reg(rs1) == read_reg(rs2); break ;
    case BRANCH_NE: do_branch = read_reg(rs1) != read_reg(rs2); break;
    case BRANCH_LT: do_branch = (int32_t)read_reg(rs1) < (int32_t) read_reg(rs2); break ;
    case BRANCH_GE: do_branch = (int32_t)read_reg(rs1) >= (int32_t) read_reg(rs2); break;
    case BRANCH_LTU: do_branch = read_reg(rs1) < read_reg(rs2); break;
    case BRANCH_GEU: do_branch = read_reg(rs1) >= read_reg(rs2); break;
    default: interrupt = INT_ILLEGAL_INSTR; // error
    }

    if (do_branch) {
        // NOTE: order of bits, added a zero bit at the end; signed offset
        unsigned int unsigned_offset = ((imm7 & 0x40) << 6) | ((imm5 & 0x1) << 11) |
            ((imm7 & 0x3f) << 5) | (imm5 & 0x1e) ;  
        int offset = sign_extend(unsigned_offset , 13);
        int ret =  pc+offset ;
    //    printf("B: pc=%x , sub3=%x , rs1=%x , rs2=%x , imm5=%x , imm7=%x , offset=%x , ret=%x\n", pc , sub3, rs1, rs2, imm5,
     //       imm7, offset, ret);
        return ret;
    } else {
        return -1 ;
    }
}

// JAL opcode: returns next PC
int jal_op(int rd, unsigned int imm)
{
    write_reg(rd, pc + 4);
    // NOTE: bit position, add 0 bit , sign extend
    unsigned int ret = pc + sign_extend(((imm & 0x80000) | ((imm & 0xff) << 11) | 
        ((imm & 0x100) << 2) | ((imm & 0x7fe00) >> 9))<<1, 21);
   // printf("JAL: pc=%x , rd=%x , imm=%x , next=%x\n", pc , rd, imm, ret);
    return ret;
}


// JALR opcode: returns next PC
int jalr_op(int rd, int rs1, unsigned int imm12)
{
    // NOTE: write register afterwards, rd could be same as rs1
    uint32_t saved_pc = pc + 4;
    // NOTE: sign extend , zero LSB
    unsigned int ret =  (read_reg(rs1) + sign_extend(imm12, 12)) & 0xfffffffe;
    //printf("JALR: pc=%x , rd=%x , rs1=%x , imm12=%x , next=%x\n", pc , rd, rs1, imm12 ,  ret);
    write_reg(rd, saved_pc);
    return ret;
}

// AUIPC opcode
int auipc_op(int rd, unsigned int imm20)
{
    write_reg(rd, pc + (imm20 << 12));
    return 0;
}

// LUI opcode
int lui_op(int rd, unsigned int imm20)
{
    write_reg(rd, imm20 << 12);
    return 0;
}



// CSR, ecall/ebreak, mret/sret, wfi
// return next_pc ;
uint32_t ecall_op(int sub3 , int sub7 , uint32_t rs1 , uint32_t rd , uint32_t imm12)
{
    switch (sub3) {
    case SYSTEM_ECALL: {    // ecall , ebreak , mret
        switch (imm12) {    // NOTE: use entire 12-bit to distinguish among the different instructions
        case ECALL_ECALL: {
            interrupt = mode+INT_UCALL;   // interrupt number is different for different modes
            break;
        }
        case ECALL_EBREAK: {
            interrupt = INT_BREAKPOINT;
            //printf("ebreak: pc=0x%x , cycle=0x%x\n", pc, no_cycles);
            break;
        }
        case ECALL_MRET: {  
            uint32_t mstatus = read_CSR(CSR_MSTATUS);
                    uint32_t prev_mode = mode;  // we are swapping mode and mstatus.mpp here
                    mode = (mstatus & CSR_MSTATUS_MPP) >> 11;  // restore CPU mode from MPP ;
                    // mie = mpie ; mpie=1 ; mpp = mode
                    write_CSR(CSR_MSTATUS, (prev_mode << 11) | CSR_MSTATUS_MPIE | ((mstatus & CSR_MSTATUS_MPIE) >> 4));
                    uint32_t next_pc = read_CSR(CSR_MEPC);
                    // printf("MRET: pc=0x%x, cycles=0x%x , next_pc=0x%x\n", pc, no_cycles , next_pc);
                    return next_pc;
                }
        case ECALL_SRET: {
            uint32_t sstatus = read_CSR(CSR_SSTATUS);
                uint32_t prev_mode = mode;  // we are swapping mode and mstatus.mpp here
                mode = (sstatus & CSR_SSTATUS_SPP) >> 8;  // restore CPU mode from SPP (NOTE: one bit only) ;
                // mie = mpie ; mpie=1 ; spp = mode[0]
                write_CSR(CSR_SSTATUS, ((prev_mode&1) << 8) | CSR_SSTATUS_SPIE | ((sstatus & CSR_SSTATUS_SPIE) >> 4));
                uint32_t next_pc = read_CSR(CSR_SEPC);
                printf("SRET: pc=0x%x, cycles=0x%x , next_pc=0x%x\n", pc, no_cycles , next_pc);
                return next_pc;
            
        }
        case ECALL_WFI: {
            write_CSR(CSR_MSTATUS, read_CSR(CSR_MSTATUS) | CSR_MSTATUS_MIE);
            wfi = 1;
            break;
        }
        default: interrupt = INT_ILLEGAL_INSTR; break;
        }
        break;
    }      
    // NOTE: order of register read/write as rd&rs1 can be the same register; special cases when register number is 0
    // atomic read CSR into rd and write CSR from rs1
    case SYSTEM_CSRRW: {
        uint32_t new_value = read_reg(rs1);
        // if x0, do not read CSR, but still write CSR
        if (rd != 0) {
            write_reg(rd, read_CSR(imm12));
        }
        write_CSR(imm12, new_value);
        break;
    }
    case SYSTEM_CSRRS: {
        uint32_t new_value = read_reg(rs1);
        uint32_t value = read_CSR(imm12);
        write_reg(rd, value);
        if (rs1 != 0) {
            write_CSR(imm12, value | new_value);    // TODO: unsettable bits?
        }
        break;
    }
    case SYSTEM_CSRRC: {
        uint32_t new_value = read_reg(rs1);
        uint32_t value = read_CSR(imm12);
        write_reg(rd, value);
        if (rs1 != 0) {
            write_CSR(imm12, value & ~new_value);    // TODO: unsettable bits?
        }
        break;
    }
    case SYSTEM_CSRRWI: {
        // if x0, do not read CSR, but still write CSR
        if (rd != 0) {
            write_reg(rd, read_CSR(imm12));
        }
        write_CSR(imm12, rs1);  // rs1 is imm5
        break;
    }
    case SYSTEM_CSRRSI: {
        uint32_t value = read_CSR(imm12);
        write_reg(rd, value);
        if (rs1 != 0) {
            write_CSR(imm12, value | rs1);    // TODO: unsettable bits?
        }
        break;
    }
    case SYSTEM_CSRRCI: {
        uint32_t value = read_CSR(imm12);
            write_reg(rd, value);
        if (rs1 != 0) {
            write_CSR(imm12, value & ~rs1);    // TODO: unsettable bits?
        }
        break;
    }
    default: interrupt = INT_ILLEGAL_INSTR ;    // undefined sub3
    }
    return -1;  // only MRET does a jump
}


// atomic memory access instructions
void atomic_op(int sub7 , int rd , int rs1 , int rs2)
{
    uint32_t data, data2 ,  addr;

    int lrsc = (sub7 & 0x8);    //bit 28 (bit 3 of sub7) covers both instructions ;

    // NOTE: register access order critical as rd&rs1 can be the same register
    addr = read_reg(rs1);   // memory address
    if (!lrsc) {
        rw_memory(MEM_READ, addr, MEM_WORD, &data);    // one piece of data in memory 
        data2 = read_reg(rs2);  // the other piece in register
    }

    switch (sub7 >> 4) {
    case AMO_ADD: 
        switch ((sub7 & 0xc)>>2) {
        case AMO_ADD_ADD: data2 += data; break;
        case AMO_ADD_SWAP: break;
        case AMO_ADD_LR: reservation = addr>>3; rw_memory(MEM_READ, addr, MEM_WORD, &data); write_reg(rd, data); break;
        case AMO_ADD_SC: 
            if (reservation == (addr >>3)) {    // check to see if lr & sc match on address, i.e. reservation still there
                data = read_reg(rs2);
                rw_memory(MEM_WRITE, addr, MEM_WORD, &data);
                write_reg(rd, 0);
                reservation = 0;
            } else {
                write_reg(rd, 1);   // TODO: do we need to reset reservation?
            } break;
        default: interrupt = INT_ILLEGAL_INSTR; break;
        } 
        break ;
    case AMO_XOR: data2 ^= data ; break;
    case AMO_AND: data2 &= data; break;
    case AMO_OR: data2 |= data; break;
    case AMO_MIN: data2 = ((int32_t)data < (int32_t)data2) ? data : data2; break;
    case AMO_MAX: data2 = ((int32_t)data < (int32_t)data2) ? data2:data;  break;
    case AMO_MINU: data2=(data<data2)?data:data2;  break;
    case AMO_MAXU: data2=(data<data2)?data2:data; break;
    default: interrupt = INT_ILLEGAL_INSTR; break;
    }

    if (!lrsc) {
        // after arithmetic operation, update memory and register
        rw_memory(MEM_WRITE, addr, MEM_WORD, &data2);
        write_reg(rd, data);
    }
}


uint32_t execute_one_instruction()
{
    uint32_t instr , opcode, sub3, sub7, rs1, rs2, rd, imm12, imm5, imm7, imm20;
    uint32_t next_pc = -1;
    unsigned int mem_data;

    // fetch instruction 
    rw_memory(MEM_INSTR, pc, MEM_WORD, &instr);

    // decode instruction
    opcode = (instr & OPCODE_MASK) >> OPCODE_SHIFT;
    sub3 = (instr & FUNCT3_MASK) >> FUNCT3_SHIFT;
    sub7 = (instr & FUNCT7_MASK) >> FUNCT7_SHIFT;
    rs1 = (instr & RS1_MASK) >> RS1_SHIFT;
    rs2 = (instr & RS2_MASK) >> RS2_SHIFT;
    imm12 = (instr & IMM12_MASK) >> IMM12_SHIFT;
    imm5 = (instr & IMM5_MASK) >> IMM5_SHIFT;
    imm7 = (instr & IMM7_MASK) >> IMM7_SHIFT;
    imm20 = (instr & IMM20_MASK) >> IMM20_SHIFT;
    rd = (instr & RD_MASK) >> RD_SHIFT;

    switch (opcode) {
    case OP_ADD: reg_op(rd, rs1, rs2, sub3, sub7); break;
    case OP_ADDI: imm_op(rd, rs1, sub3, sub7, sign_extend(imm12, 12)); break;
        // NOTE: memory address offset is signed
    case OP_LB:
        rw_memory(MEM_READ, read_reg(rs1) + sign_extend(imm12, 12), sub3, &mem_data);
        write_reg(rd, mem_data);
        break;
    case OP_SB:
        mem_data = read_reg(rs2);
        rw_memory(MEM_WRITE, read_reg(rs1) + sign_extend((imm7 << 5) | imm5, 12), sub3, &mem_data);
        break;
    case OP_BEQ: next_pc = branch_op(rs1, rs2, sub3, imm5, imm7); break;
    case OP_JAL: next_pc = jal_op(rd, imm20); break;
    case OP_JALR: next_pc = jalr_op(rd, rs1, imm12);  break;
    case OP_AUIPC: auipc_op(rd, imm20);  break;
    case OP_LUI: lui_op(rd, imm20);  break;
    case OP_ECALL: next_pc = ecall_op(sub3, sub7, rs1, rd, imm12); break;
    case OP_FENCEI: break; // TODO: don't need to anything until we have cache or pipeline
    case OP_A: atomic_op(sub7, rd, rs1, rs2); break;
    default: interrupt = INT_ILLEGAL_INSTR; break;  // invalid opcode
    }

    return next_pc;
}


// return next_pc; -1 if we don't take the interrupt for some reason (currently not possible)
// possible interrupts: timer + exceptions
uint32_t execute_interrupt(uint32_t interrupt)
{
    uint32_t result = -1;

    uint32_t mstatus = read_CSR(CSR_MSTATUS);
    uint32_t exception_delegate = read_CSR(CSR_MEDELEG);
    uint32_t interrupt_delegate = read_CSR(CSR_MIDELEG);
    int delegated = 0;

    if (interrupt & 0x80000000) {
        int interrupt_no = interrupt & 0x7fffffff;
        delegated = (interrupt_no < 32) && (interrupt_delegate & (1 << interrupt_no));
    }
    else {
        delegated = (interrupt < 32) && (exception_delegate & (1 << interrupt));
    }

    // default is M mode
    // NOTE: mstatus.mie, mtip, mtie all already checked when interrupt was assigned
    if (mode!=MODE_M || !delegated) {
        write_CSR(CSR_MCAUSE, interrupt);
        // mstatus: copy MIE to MPIE, clear MIE, copy current mode into MPP
        write_CSR(CSR_MSTATUS, ((read_CSR(CSR_MSTATUS) & CSR_MSTATUS_MIE) << 4) | (mode << 11));
        write_CSR(CSR_MTVAL, (interrupt & 0x80000000) ? 0 : pc);   // TODO: can provide diff info for certain types of interrupts 
        write_CSR(CSR_MEPC, pc);    // NOTE: interrupt and exception cases are different, but both should save the current pc
        mode = MODE_M; // switch to M mode ;
        result = read_CSR(CSR_MTVEC);  // jump to interrupt routine, no vectoring support yet
 //       printf("[time=0x%usus]M INTR: pc=%x , interrupt=%x , next=%x\n", (uint32_t)get_microseconds() , pc, interrupt, result);
    }
    else {  // trap to S mode
        write_CSR(CSR_SCAUSE, interrupt);
        // mstatus: copy SIE to SPIE, clear SIE, copy current mode(one bit) into SPP
        write_CSR(CSR_SSTATUS, ((read_CSR(CSR_SSTATUS) & CSR_SSTATUS_SIE) << 4) | ((mode & 1) << 8));
        write_CSR(CSR_STVAL, (interrupt & 0x80000000) ? 0 : pc);   // TODO: can provide diff info for certain types of interrupts 
        write_CSR(CSR_SEPC, pc);    // NOTE: interrupt and exception cases are different, but both should save the current pc
        mode = MODE_S; // switch to M mode ;
        result = read_CSR(CSR_STVEC);  // jump to interrupt routine, no vectoring support yet
        printf("[time=0x%usus]S INTR: pc=%x , interrupt=%x , next=%x\n", (uint32_t)get_microseconds() , pc, interrupt, result);
    }
    return result;
}


int execute_code()
{ 
    unsigned int next_pc;

    for (;;) {
        next_pc = -1;  // assume no jump
        interrupt = 0xffffffff;     // NOTE: 0 is valid interrupt
        no_cycles++;

        // run clint every 1024 instructions
        if ((no_cycles & 0x3ff) == 0) {
            interrupt = run_clint();
        } 

        // 4 combination of interrupt & wfi  
        if (interrupt==0xffffffff) {   // if no interrupt; if there is interrupt, will execute the interrupt-handling code following this if
            if (wfi) {
                continue ;   // loop back to see if there is pending interrupt, specifically skip the pc increment
            }
            else {
                next_pc = execute_one_instruction();    // does not change any state except for what is defined in the instruction
            }
        }
        // NOTE: mie already checked when generating interrupt
        if (interrupt!=0xffffffff) {
            next_pc = execute_interrupt(interrupt);
        }
        if (next_pc == -1) next_pc = pc + 4;    // next instruction if there is no jump; only executed for !intetrrupt&&!wfi case
        pc = next_pc;
    }
}

// load machine from file into memory starting at 0
int load_code(char *file_name)
{
    // read instructions and save into memory
    FILE *f = fopen (file_name , "rb") ;
    int len;
    
    if (f == NULL) {
        printf("file open error: %s\n", file_name);
        exit(1);
    }

    uint8_t *p = mem ;

    fseek(f, 0, SEEK_END) ;
    len = ftell(f) ; 
    rewind(f) ;
    fread(p, len, 1, f) ;
    fclose(f) ;

    return TRUE;
}

// load DTB file
int load_dtb(char* file_name)
{
    FILE *f = fopen(file_name, "rb");
    if (!f || ferror(f))
    {
        fprintf(stderr, "Error: DTB file \"%s\" not found\n", file_name);
        return -5;
    }
    fseek(f, 0, SEEK_END);
    long dtblen = ftell(f);
    fseek(f, 0, SEEK_SET);
    dtb_offset = MEMSIZE - dtblen ;  // do we need to store core structure in memory?
    if (fread(mem + dtb_offset, dtblen, 1, f) != 1)
    {
        fprintf(stderr, "Error: Could not open dtb \"%s\"\n", file_name);
        return -9;
    }
    fclose(f);
    return 0;
}


// debugging syscall on I/O write: use x5: 1 halt 10 print all registers 11 print string (ptr in x6) 12 print integer (value in x6) 
void debug_syscall() {

    int call_no = read_reg(5);

    switch (call_no) {
    case 1: printf("total_cycles = %d", no_cycles); exit(0); break;
    case 10:
        printf("0x%x ", pc);
        for (int i = 1; i < 32; i++) {
            printf("0x%x ", read_reg(i));
        }
        printf("\n");
        break;
    case 11: printf("%s", (char*)(read_reg(6))); break;
    case 12:printf("0x%x", read_reg(6)); break;
    default: assert(0);     // unsupported system call
        break;
    }
}


// takes two optional argument: machine code file name & dtb file name
int main(int argc, char** argv)
{
    load_code((argc<=1)?DEFAULT_FILE:argv[1]);
    if (argc >= 2) load_dtb(argv[2]) ;

    // initialize CPU state
    pc = INITIAL_PC;
    mode = MODE_M; // Machine-mode.
    no_cycles = 0;

    init_clint();   

    // a10 and a11 needed for Linux
    write_reg(10, 0x0); // hart ID
    write_reg(11, dtb_offset + INITIAL_PC); // DTB address in memory
 
    execute_code();
}
