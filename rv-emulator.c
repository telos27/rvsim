#include <stdio.h>
#include <stdint.h>
#include <assert.h>

// initial instruction address upon startup
#define INITIAL_PC 0

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

// opcodes: RV32I complete
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

// R/I instruction funct3
#define ALU_ADD 0x0     // SUB funct7=0x20
#define ALU_XOR 0x4
#define ALU_OR 0x6
#define ALU_AND 0x7
#define ALU_SLL 0x1
#define ALU_SRL 0x5     // SRA/SRAI funct7=0x20
#define ALU_SLT 0x2
#define ALU_SLTU 0x3

// funct7: SUB & SRA
#define NORMAL 0x0
#define SUB 0x20
#define SRA 0x20

// memory operations funct3
#define MEM_BYTE 0x0
#define MEM_HALFWORD 0x1
#define MEM_WORD 0x2
#define MEM_UBYTE 0x4
#define MEM_UHALFWORD 0x5

// branch funct3
#define BRANCH_EQ 0x0
#define BRANCH_NE 0x1
#define BRANCH_LT 0x4
#define BRANCH_GE 0x5
#define BRANCH_LTU 0x6
#define BRANCH_GEU 0x7

// usable memory size: 16M
#define MEMSIZE 16*1024*1024  

// machine code file name, if unspecificed
#define DEFAULT_FILE "asm.o"


#define BOOL int8_t
#define TRUE 1
#define FALSE 0 
#define REGISTER uint32_t   // enough to hold the content of a single register


// external memory
static uint8_t mem[MEMSIZE];   // main memory


// internal state
static REGISTER regs[32];
static unsigned int pc ;       // 32-bit PC

static unsigned int no_cycles; // execution cycles; currently always 1 cycle/instruction

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


// memory operation, no sign extension done here
// can improve perf by handling multiple bytes at once
int mem_interface(BOOL write, unsigned int addr, int size, unsigned int* data)
{

    assert(size == MEM_BYTE || size == MEM_HALFWORD || size == MEM_WORD);
    if (write == TRUE) {
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
        default: assert(0); // TBD or error
        }

    }
    else {
        switch (size) {
        case MEM_BYTE:
            // NOTE: no sign extension
            *data = mem[addr];
            break;
        case MEM_HALFWORD:
            // NOTE: no sign extension
            *data = mem[addr] | ((unsigned int)mem[addr + 1]) << 8 ;
            break;
        case MEM_WORD:
            *data = mem[addr] | ((unsigned int)mem[addr + 1]) << 8 |
                ((unsigned int)mem[addr + 2]) << 16 | ((unsigned int)mem[addr + 3]) << 24;
            break;
        default:
            assert(0); // TBD or error
        }

    }
    return TRUE;
}


// read/write memory, instruction semantics, which includes sign extension in some cases
// TODO: handle unsigned R/W 
// NOTE: little-endian
int rw_memory(BOOL write , unsigned int addr , int sub3 , unsigned int *data)
{

    assert(sub3==MEM_BYTE || sub3==MEM_HALFWORD || sub3==MEM_WORD) ;
    if (write==TRUE) {
        return mem_interface(write, addr, sub3, data);
    } else {
        uint32_t read_data;
        int result = mem_interface(write, addr, sub3, &read_data);

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
                *data = read_data ;
                break ;
            default: 
                assert(0); // TBD or error
        }
        return result;

    }
}

// TODO: make sure signedness is handled correctly
int reg_op(int rd , int rs1 , int rs2 , int sub3 , int sub7)
{
    switch (sub3) {
    case ALU_ADD: 
        if (sub7 == NORMAL) {
            write_reg(rd , read_reg(rs1) + read_reg(rs2)) ;
        }
        else if (sub7 == SUB) {
            write_reg(rd , read_reg(rs1) - read_reg(rs2));
        }
        else {
            assert(0);  // nonexistent sub7
        }
        break;
    case ALU_XOR: write_reg(rd ,  read_reg(rs1) ^ read_reg(rs2)); break;
    case ALU_OR: write_reg(rd, read_reg(rs1) | read_reg(rs2)); break;
    case ALU_AND:
        write_reg(rd , read_reg(rs1) & read_reg(rs2)) ; 
        break;
    case ALU_SLL:
        write_reg(rd , read_reg(rs1) << read_reg(rs2)); break;
    case ALU_SRL:
        if (sub7 ==0x0) {
            write_reg(rd , read_reg(rs1) >> read_reg(rs2)) ; 
        }
        else if (sub7 == 0x20) {
            // signed right shift
            write_reg(rd , ((uint32_t)read_reg(rs1)) >> read_reg(rs2)) ;
        }
        break;
    case ALU_SLT:
        write_reg(rd , (read_reg(rs1) < read_reg(rs2))?1:0) ;
        break;
        default: assert(0); // TBD or error
    }
    return TRUE;
}

int imm_op(int rd , int rs1 , int sub3 , int imm)
{
    switch (sub3) {
        case ALU_ADD: write_reg(rd ,  read_reg(rs1) + imm) ; break;
        case ALU_OR: write_reg(rd ,  read_reg(rs1) | imm) ; break;
        case ALU_AND: write_reg(rd , read_reg(rs1) & imm) ; break;
        default: assert(0); // TBD or error
    }
    return TRUE;
}

// return a sign-extended version of a number with no_bits
int sign_extend(unsigned int n , int no_bits)
{
    // TODO: is this portable?
    return (((signed int)n) << (32 - no_bits)) >> (32 - no_bits);
}

// return next PC, -1 if we don't branch
// TODO: check signedness
int branch_op(int rs1 , int rs2 , int sub3 , unsigned int imm5 , unsigned int imm7)
{
    BOOL do_branch = FALSE ;

    switch (sub3) {
    case BRANCH_EQ: do_branch = read_reg(rs1) == read_reg(rs2); break ;
    case BRANCH_NE: do_branch = read_reg(rs1) != read_reg(rs2); break;
    case BRANCH_LT: do_branch = read_reg(rs1) < read_reg(rs2); break ;
    case BRANCH_GE: do_branch = read_reg(rs1) >= read_reg(rs2); break;
    default: assert(0); // TBD or error
    }

    if (do_branch) {
        // NOTE: order of bits, added a zero bit at the end; signed offset
        unsigned int unsigned_offset = ((imm7 & 0x40) << 6) | ((imm5 & 0x1) << 11) |
            ((imm7 & 0x3f) << 5) | (imm5 & 0x1e) ;  
        int offset = sign_extend(unsigned_offset , 13);
        return pc+offset ;
    } else {
        return -1 ;
    }
}

// JAL opcode: returns next PC
int jal_op(int rd, unsigned int imm)
{
    write_reg(rd, pc + 4);
    // NOTE: bit position, add 0 bit , sign extend
    return pc + sign_extend((imm & 0xfffff) | ((imm & 0xff) << 11) | ((imm & 0x100) << 2) | ((imm & 0x7fe00) << 1), 21);
}


// JALR opcode: returns next PC
int jalr_op(int rd, int rs1, unsigned int imm12)
{
    write_reg(rd, pc + 4);
    // NOTE: sign extend , zero LSB
    return (read_reg(rs1) + sign_extend(imm12, 12)) & 0xfffffffe;
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

// print total cycles and exit
void halt_cpu()
{
    printf("total_cycles = %d", no_cycles);
    exit(0);
}

// system call
// use x5: 1 halt 10 print all registers 11 print string (ptr in x6) 12 print integer (value in x6) 
void ecall_op()
{
    int call_no = read_reg(5);

    switch (call_no) {
    case 1: halt_cpu(); break;
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


int execute_code()
{
    unsigned instr ,  opcode , sub3 , sub7 , rs1 , rs2 , rd , imm12 , imm5 , imm7 ,  imm20 ;
    unsigned int next_pc;

    unsigned int mem_data;

    for (;;) {
        next_pc = -1 ;
        no_cycles++ ;
        // fetch instruction 
        rw_memory(FALSE , pc , MEM_WORD , &instr) ;
        // decode instruction
        opcode = (instr & OPCODE_MASK)>> OPCODE_SHIFT ;
        sub3 = (instr & FUNCT3_MASK) >> FUNCT3_SHIFT ;
        sub7 = (instr & FUNCT7_MASK) >> FUNCT7_SHIFT ;
        rs1 = (instr & RS1_MASK) >> RS1_SHIFT ;
        rs2 = (instr & RS2_MASK) >> RS2_SHIFT ;
        imm12 = (instr & IMM12_MASK) >> IMM12_SHIFT ;
        imm5 = (instr & IMM5_MASK) >> IMM5_SHIFT ;
        imm7 = (instr & IMM7_MASK) >> IMM7_SHIFT ;
        imm20 = (instr & IMM20_MASK) >> IMM20_SHIFT ;
        rd = (instr & RD_MASK) >> RD_SHIFT ;
        switch (opcode) {
            case OP_ADD: reg_op(rd, rs1, rs2, sub3, sub7); break;
            case OP_ADDI: imm_op(rd, rs1, sub3, sign_extend(imm12, 12)); break;
                // NOTE: memory address offset is signed
            case OP_LB: 
                rw_memory(FALSE , read_reg(rs1)+sign_extend(imm12 , 12) , sub3 , &mem_data) ; 
                write_reg(rd, mem_data);
                break;
            case OP_SB: 
                mem_data = read_reg(rs2);
                rw_memory(TRUE, read_reg(rs1) + sign_extend((imm7 << 5) | imm5, 12), sub3, &mem_data); 
                break;
            case OP_BEQ: next_pc = branch_op(rs1 , rs2 , sub3 , imm5 , imm7) ; break ;
            case OP_JAL: next_pc = jal_op(rd, imm20); break;    
            case OP_JALR: jalr_op(rd , rs1 ,  imm12);  break;   
            case OP_AUIPC: auipc_op(rd , imm20);  break;    // TBD
            case OP_LUI: lui_op(rd , imm20) ;  break;    // TBD
            case OP_ECALL: ecall_op() ; break ;
            default: assert(0);  // unsupported opcode
        }
        if (next_pc==-1) next_pc = pc + 4 ;
        pc = next_pc ;
    }
}

int main(int argc, char** argv)
{
    load_code((argc<=1)?DEFAULT_FILE:argv[1]);
    pc = INITIAL_PC;
    no_cycles = 0;
    execute_code();
}
