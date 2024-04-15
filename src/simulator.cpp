#include <cstdio>
#include <cstring>
#include <iostream>
#include "simulator/simulator.h"

/*
 * Parameters: memory
 * Init memory, pc, register
 */
Simulator::Simulator(MemoryManager* memory) {
    this ->pc = 0;
    this ->memory = memory;
    int i = 0;
    while (i < REGNUM){
        this ->reg[i] = 0;
        i++;
    }
}

Simulator::~Simulator() {}

/* init_stack()
 * Parameters: base_addr, max_size
 * Init register, init memory (set to 0) according to stack base address and max
 */
void Simulator::init_stack(uint32_t base_addr, uint32_t max_size) {
    this->reg[2] = base_addr;
    this->max_stack_size = max_size;
    this->stack_base = base_addr;
    uint32_t current_addr = base_addr;
    uint32_t mem_hold = base_addr - max_size;
    while (current_addr > mem_hold) {
        if (memory->is_page_exit(current_addr)) {
            memory->set_byte(current_addr, 0);
        } else{
            memory->add_page(current_addr);
            memory->set_byte(current_addr, 0);
        }
        current_addr--;
    }
}
/* initializeRegister()
 * Parameters: *reg, template <typename T>
 * InitializeRegister, set memory of register with 0 and size of register.
 */
template <typename T>
void initializeRegister(T *reg) {
    memset(&reg, 0, sizeof(reg));
}

/* simulate()
 * Initialize registers of all different stages
 * Insert Bubble to later pipeline stages when fetch the first instruction
 * During the main simulation loop, we set $zero to 0 as some instruction might set this register to non-zero and check stack overflow.
 * Initialize bool flag for hazards.
 * Main simulation stages and copy old register values to new register values when not stall.
 */
void Simulator::simulate() {
    initializeRegister(&this ->f_reg);
    initializeRegister(&this ->d_reg);
    initializeRegister(&this ->e_reg);
    initializeRegister(&this ->m_reg);
    initializeRegister(&this ->f_reg_new);
    initializeRegister(&this ->d_reg_new);
    initializeRegister(&this ->e_reg_new);
    initializeRegister(&this ->m_reg_new);

    // insert Bubble to later pipeline stages when fetch the first instruction
    f_reg.bubble = d_reg.bubble = e_reg.bubble = m_reg.bubble = true;
    //main simulation loop
    while (true) {
        //std::cout << "----Cycle-----" << history.cycle_count << std::endl;

        this ->history.cycle_count++;
        // set $zero to 0, some instruction might set this register to non-zero
        bool flag_r0 = this -> reg[0] == 0;
        if (!flag_r0){
            this -> reg[0] = 0;
        }

        // check stack overflow
        uint32_t stack_size = this -> stack_base - this -> max_stack_size;
        bool overflow_flag = reg[2] >= stack_size;
        if (!overflow_flag){
            std::cout << "overflow" ;
        }

        this ->execute_write_back = false;
        this ->execute_wb_reg = -1;
        this ->memory_write_back = false;
        this ->memory_wb_reg = -1;

        //std::cout << "a0" <<reg[10] << std::endl;

        this->fetch();
        this->decode();
        this->excecute();
        this->memory_access();
        this->write_back();

        // copy old register values to new register values
        if (this ->f_reg.stall == 0) {
            this ->f_reg = this -> f_reg_new;
        } else {
            this ->f_reg.stall--;
            //this ->history.stall_cycle_count++;
        }

        if (this ->d_reg.stall == 0) {
            this ->d_reg = this -> d_reg_new;
        } else {
            this ->d_reg.stall--;
            //this ->history.stall_cycle_count++;
        }
        this ->e_reg = this ->e_reg_new;
        this ->m_reg = this ->m_reg_new;

        initializeRegister(&this ->f_reg_new);
        initializeRegister(&this ->d_reg_new);
        initializeRegister(&this ->e_reg_new);
        initializeRegister(&this ->m_reg_new);
        //this ->history.cycle_count++;

    }

}

/* fetch()
 * Update pc and f_reg_new
 */
void Simulator::fetch() {
    uint32_t instruction = this-> memory-> get_int(this ->pc);
    updateFetchRegister(instruction, 4);
    this ->pc += 4;
}

/* updateFetchRegister()
 * Parameters: inst, length
 * Update pc and f_reg_new
 */
void Simulator::updateFetchRegister(uint32_t inst, uint32_t length) {
    this ->f_reg_new.stall = 0;
    this ->f_reg_new.bubble = false;
    this ->f_reg_new.len = length;
    this ->f_reg_new.inst = inst;
    this ->f_reg_new.pc = this ->pc;
}

/* extractBit()
 * Parameters: inst, start positions, end positions.
 * Calculate the length of the field.
 * Create a mask that contains only 1 of the bits of the field length and move it to the correct position.
 * Move the mask left by s bits and use it to extract the field, extract the field and right-justify it.
 */
unsigned int extractBit(unsigned int inst, int s, int e) {
    unsigned int length = e - s + 1;
    unsigned int mask = (1U << length) - 1;
    mask <<= s;
    unsigned int field = (inst & mask) >> s;
    return field;
}

/* sign_extension()
 * Parameters: src instrcutions, bit length.
 * Check if the sign bit is 1 (negative).
 * If the sign bit is 1, use a mask to set all bits above the sign bit to 1 for sign expansion,
 * Since we are starting with unsigned int and then manipulating it, the final result is naturally 32-bit
 */
unsigned int sign_extension(unsigned int src, int bit) {
    int sign_bit = (src >> bit) & 1;
    if (sign_bit) {
        unsigned int mask = 0xFFFFFFFF << bit;
        src |= mask;
    }
    return src;
}

/* extract_R_inst()
 * Parameters: reg1, reg2, func3, func7, rs1, rs2, rd, aluCal, op1, op2, dest
 * Based on the func3 and func7 code, determine the type of the aluCal.
 * Determine the op1, op2, reg1, reg2, dest based on the extracted instructions.
 */
void Simulator::extract_R_inst(uint32_t &reg1, uint32_t &reg2, uint32_t func3, uint32_t func7, uint32_t rs1, uint32_t rs2, uint32_t rd, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &dest) {
    op1 = this ->reg[rs1];
    op2 = this ->reg[rs2];
    reg1 = rs1;
    reg2 = rs2;
    dest = rd;
    if (func3 == 0x0) {
        if (func7 == 0x00) { // add rd, rs1, rs2
            aluCal = ADD;
        } else if (func7 == 0x20) { // sub rd, rs1, rs2
            aluCal = SUB;
        }
    } else if (func3 == 0x1) {
        if (func7 == 0x00) { // sll rd, rs1, rs2
            aluCal = SLL;
        }
    } else if (func3 == 0x2 && func7 == 0x00) { // slt rd, rs1, rs2
        aluCal = SLT;
    } else if (func3 == 0x3 && func7 == 0x00){
        aluCal = SLTU;
    }
    else if (func3 == 0x4) {
        if (func7 == 0x00) { // xor rd, rs1, rs2
            aluCal = XOR;
        }
    } else if (func3 == 0x5) {
        if (func7 == 0x00) { // srl rd, rs1, rs2
            aluCal = SRL;
        } else if (func7 == 0x20) { // sra rd, rs1, rs2
            aluCal= SRA;
        }
    } else if (func3 == 0x6) {
        if (func7 == 0x00) { // or rd, rs1, rs2
            aluCal = OR;
        }
    } else if (func3 == 0x7 && func7 == 0x00) { // and rd, rs1, rs2
        aluCal = AND;
    }
}

/* extract_I_imm()
 * Parameters: reg1, funct3, funct7, imm_i, rs1, rd, aluCal, op1, op2, dest
 * For immediate type instructions, determine the operation by funct3.
 * Set the op1 to the value of rs1 register and op2 to immediate value imm_i.
 * Set the reg1 to the rs1 and dest to rd register.
 * Use the funct7 field to distinguish between SRLI/SRAI.
 */
void Simulator::extract_I_imm(uint32_t &reg1, uint32_t funct3, uint32_t funct7, int32_t imm_i, uint32_t rs1, uint32_t rd, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &dest) {
    op1 = this ->reg[rs1];
    op2 = imm_i;
    reg1 = rs1;
    dest = rd;
    if (funct3 == 0x0) {
        aluCal = ADDI;
    } else if (funct3 == 0x1){
        aluCal = SLLI;
        op2 = op2 & 0x1F;
    } else if (funct3 == 0x2) {
        aluCal = SLTI;
    } else if (funct3 == 0x3) {
        aluCal = SLTIU;
    } else if (funct3 == 0x4) {
        aluCal = XORI;
    } else if (funct3 == 0x5){
        if (funct7 == 0x0){
            aluCal = SRLI;
            op2 = op2 & 0x1F;
        }
        else if (funct7 == 0x20){ // srai rd, rs1, imm
            aluCal = SRAI;
            op2 = op2 & 0x1F;
        }
    }
    else if (funct3 == 0x6) {
        aluCal = ORI;
    } else if (funct3 == 0x7) {
        aluCal = ANDI;
    }
}

/* extract_I_load()
 * Parameters: reg1, func3, imm_i, rs1, rd, aluCal, op1, op2, offset, dest
 * For load type instructions, determine the load operation by func3.
 * Set the op1 to the value of rs1 register and op2 to immediate value imm_i.
 * The offset for the memory load is set to the immediate value imm_i.
 * Set the reg1 to the rs1 and dest to rd register.
 */
void Simulator::extract_I_load(uint32_t &reg1, uint32_t func3, int32_t imm_i, uint32_t rs1, uint32_t rd, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &offset, uint32_t &dest){
    op1 = this ->reg[rs1];
    op2 = imm_i;
    offset = imm_i;
    reg1 = rs1;
    dest = rd;

    if (func3 == 0x0) {
        aluCal = LB;
        //std::cout << "op1:" << op1 << "op2:" << op2 << std::endl;
    } else if (func3 == 0x1) {
        aluCal = LH;
    } else if (func3 == 0x2) {
        aluCal = LW;
    } else if (func3 == 0x4) {
        aluCal = LBU;
    } else if (func3 == 0x5) {
        aluCal = LHU;
    }
}

/* extract_I_LUI()
 * Parameters: rd, imm_LUI, aluCal, op1, op2, offset, dest
 * Handle the LUI (Load Upper Immediate) instruction.
 * Set the op1 to the immediate value imm_LUI and op2 to zero.
 * The offset for the upper load is set to the immediate value imm_LUI.
 * Set the destination register to rd.
 */
void Simulator::extract_I_LUI(uint32_t rd, int32_t imm_LUI, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &offset, uint32_t &dest){
    op1 = imm_LUI;
    op2 = 0;
    dest = rd;
    offset = imm_LUI;
    aluCal = LUI;
}

/* extract_I_AUIPC()
 * Parameters: rd, imm_auipc, aluCal, op1, op2, offset, dest
 * Handle the AUIPC (Add Upper Immediate to PC) instruction.
 * Set the op1 to the immediate value imm_auipc and op2 to zero.
 * The offset is set to the immediate value imm_auipc.
 * Set the destination register to rd.
 */
void Simulator::extract_I_AUIPC(uint32_t rd, int32_t imm_auipc, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &offset, uint32_t &dest){
    op1 = imm_auipc;
    op2 = 0;
    dest = rd;
    offset = imm_auipc;
    aluCal = AUIPC;
}

/* extract_S()
 * Parameters: reg1, reg2, func3, rs1, rs2, imm_s, aluCal, op1, op2, offset
 * For store type instructions, determine the store operation by func3.
 * Set the op1 to the value of rs1 register and op2 to the value of rs2 register.
 * The offset for the memory store is set to the immediate value imm_s.
 * Set the reg1 to the rs1 and reg2 to rs2.
 */
void Simulator::extract_S(uint32_t &reg1, uint32_t &reg2, uint32_t func3, uint32_t rs1, uint32_t rs2, int32_t imm_s, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &offset){
    op1 = this->reg[rs1];
    op2 = this->reg[rs2];
    reg1 = rs1;
    reg2 = rs2;
    offset = imm_s;
    if (func3 == 0x00) {
        aluCal = SB;
    } else if (func3 == 0x01) {
        aluCal = SH;
    } else if (func3 == 0x02) {
        //std::cout << "here" << op2 << std::endl;
        aluCal = SW;
    }
}

/* extract_B()
 * Parameters: reg1, reg2, func3, rs1, rs2, imm_b, aluCal, op1, op2, offset
 * For branch type instructions, determine the branch operation by func3.
 * Set the op1 to the value of rs1 register and op2 to the value of rs2 register.
 * The offset for the branch is set to the immediate value imm_b.
 * Set the reg1 to the rs1 and reg2 to rs2.
 */
void Simulator::extract_B(uint32_t &reg1, uint32_t &reg2, uint32_t func3, uint32_t rs1, uint32_t rs2, int32_t imm_b, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &offset){
    reg1 = rs1;
    reg2 = rs2;
    op1 = this->reg[rs1];
    op2 = this->reg[rs2];
    offset = imm_b;
    if (func3 == 0x0) {
        aluCal = BEQ;
    } else if (func3 == 0x1) {
        aluCal = BNE;
    } else if (func3 == 0x4) {
        aluCal = BLT;
    } else if (func3 == 0x5) {
        aluCal = BGE;
    } else if (func3 == 0x6) {
        aluCal = BLTU;
    } else if (func3 == 0x7) {
        aluCal = BGEU;
    }

}

/* extract_ECALL()
 * Parameters: reg1, reg2, aluCal, op1, op2, dest
 * Handle the ECALL (System Call) instruction.
 * Set the op1 and op2 to the value of registers used for system call numbers and parameters.
 * The destination register is set by convention for return values from system calls.
 */
void Simulator::extract_ECALL(uint32_t &reg1, uint32_t &reg2, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &dest){
    op1 = this->reg[10];
    op2 = this->reg[17];
    reg1 = 10;
    reg2 = 17;
    dest = 10;
    aluCal = ECALL;
}

/* extract_jalr()
 * Parameters: reg1, imm_jalr, rs1, rd, aluCal, op1, op2, dest
 * Handle the JALR (Jump And Link Register) instruction.
 * Set the op1 to the value of rs1 register and op2 to immediate value imm_jalr.
 * Set the reg1 to rs1 and destination register to rd.
 */
void Simulator::extract_jalr(uint32_t &reg1, int32_t imm_jalr, uint32_t rs1, uint32_t rd, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &dest){
    op1 = this->reg[rs1];
    op2 = imm_jalr;
    reg1 = rs1;
    dest = rd;
    aluCal = JALR;
}

/* extract_jal()
 * Parameters: rd, imm_jal, aluCal, op1, op2, offset, dest
 * Handle the JAL (Jump And Link) instruction.
 * Set the offset to the immediate value imm_jal, op1 to immediate value imm_jal and op2 to zero.
 * The destination register is set to rd.
 */
void Simulator::extract_jal(uint32_t rd, int32_t imm_jal, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &offset, uint32_t &dest){
    offset = imm_jal;
    op1 = imm_jal;
    op2 = 0;
    dest = rd;
    aluCal = JAL;
}

/* decode()
 * Decode the fetched instruction and determine its type based on the opcode.
 * Handle stalls and bubbles in the pipeline by checking the fetch register.
 * Extract the fields from the instruction and pass them to the appropriate extract function based on the instruction type.
 * Set up the decode register for the next pipeline stage with the extracted instruction details.
 */
void Simulator::decode() {
    // Handle pipeline stall
    if (this ->f_reg.stall > 0){
        this ->pc -= 4;
        return;
    }

    // Check for bubbles or no-op instructions
    bool flag_bubble = this ->f_reg.bubble == true || this ->f_reg.inst == 0;
    if (flag_bubble){
        this ->d_reg_new.bubble = true;
        return;
    }

    // Instruction and its fields
    uint32_t inst = this ->f_reg.inst;
    uint32_t opcode = 0;
    uint32_t func3 = 0, func7 = 0;
    uint32_t rs1 = 0, rs2 = 0, rd = 0;
    int32_t imm_i, imm_lui, imm_s, imm_b, imm_jalr, imm_jal, imm_auipc;
    uint32_t op1 = 0, op2 = 0, offset = 0;
    uint32_t destination = 0;
    uint32_t reg1 = 0, reg2 = 0;
    Instruction aluCal = Instruction::UNKNOWN;

    opcode = extractBit(inst, 0, 6);
    //R-type instruction
    if(opcode == OP_REG){
        rd = extractBit(inst, 7, 11);
        func3 = extractBit(inst, 12, 14);
        rs1 = extractBit(inst, 15, 19);
        rs2 = extractBit(inst, 20, 24);
        func7 = extractBit(inst, 25, 31);
        extract_R_inst(reg1, reg2, func3, func7, rs1, rs2, rd, aluCal, op1, op2, destination);
    }
    //I-type immediate
    else if (opcode == OP_IMM){
        rd = extractBit(inst, 7, 11);
        func3 = extractBit(inst, 12, 14);
        rs1 = extractBit(inst, 15, 19);
        //other ones
        imm_i = int32_t(inst) >> 20;
        func7 = extractBit(inst, 25, 31);
        extract_I_imm(reg1, func3, func7, imm_i, rs1, rd, aluCal, op1, op2, destination);
    }
    //I-type LUI
    else if (opcode == OP_LUI){
        rd = extractBit(inst, 7, 11);
        imm_lui = int32_t (extractBit(inst, 12, 31));
        extract_I_LUI(rd, imm_lui, aluCal, op1, op2, offset, destination);
    }
    //I-type AUIPC
    else if (opcode == OP_AUIPC){
        rd = extractBit(inst, 7, 11);
        imm_auipc = int32_t (extractBit(inst, 12, 31));
        extract_I_AUIPC(rd, imm_auipc, aluCal, op1, op2, offset, destination);
    }
    //I-type load
    else if (opcode == OP_LOAD){
        rd = extractBit(inst, 7, 11);
        func3 = extractBit(inst, 12, 14);
        rs1 = extractBit(inst, 15, 19);
        imm_i = int32_t(inst) >> 20;
        extract_I_load(reg1, func3, imm_i, rs1, rd, aluCal, op1, op2, offset, destination);
    }
    //S-type
    else if (opcode == OP_STORE){
        int imm5 = extractBit(inst, 7, 11);
        int imm7 = extractBit(inst, 25, 31);
        int imm_temp = (imm7 << 5) | imm5;
        func3 = extractBit(inst, 12, 14);
        rs1 = extractBit(inst, 15, 19);
        rs2 = extractBit(inst, 20, 24);
        int32_t imm_s = sign_extension(imm_temp, 11);
        extract_S(reg1, reg2, func3, rs1, rs2, imm_s, aluCal, op1, op2, offset);
    }
    //B-type
    else if (opcode == OP_BRANCH){
        func3 = extractBit(inst, 12, 14);
        rs1 = extractBit(inst, 15, 19);
        rs2 = extractBit(inst, 20, 24);
        int imm1 = extractBit(inst, 7, 7);
        int imm4 = extractBit(inst, 8, 11);
        int imm6 = extractBit(inst, 25, 30);
        int imm31 = extractBit(inst, 31, 31);
        int tmp_imm = (imm4 << 1) | (imm6 << 5) | (imm1 << 11) | (imm31 << 12);
        imm_b = sign_extension(tmp_imm, 12);
        extract_B(reg1, reg2, func3, rs1, rs2, imm_b, aluCal, op1, op2, offset);
    }
    //ECALL
    else if (opcode == OP_SYSTEM){
        func3 = extractBit(inst, 12, 14);
        func7 = extractBit(inst, 25, 31);
        if (func3 == 0x0 && func7 == 0x000) {
            extract_ECALL(reg1, reg2, aluCal, op1, op2, destination);
        }
    }
    //JAL
    else if (opcode == OP_JAL){
        rd = extractBit(inst, 7, 11);
        int imm8 = extractBit(inst, 12, 19);
        int imm1 = extractBit(inst, 20, 20);
        int imm10 = extractBit(inst, 21, 30);
        int imm31 = extractBit(inst, 31, 31);
        int tmp_imm = (imm8 << 12) | (imm1 << 11) | (imm10 << 1) | (imm31 << 20);
        imm_jal = sign_extension(tmp_imm, 20);
        extract_jal(rd, imm_jal, aluCal, op1, op2, offset, destination);
    }
    //JALR
    else if (opcode == OP_JALR){
        rd = extractBit(inst, 7, 11);
        func3 = extractBit(inst, 12, 14);
        rs1 = extractBit(inst, 15, 19);
        imm_jalr = int32_t(inst) >> 20;
        extract_jalr(reg1,imm_jalr, rs1, rd, aluCal, op1, op2, destination);
    }

    // Update new decode register with extracted and decoded instruction details
    this-> d_reg_new.stall = false;
    this-> d_reg_new.bubble = false;
    this-> d_reg_new.rs1 = reg1;
    this-> d_reg_new.rs2 = reg2;
    this-> d_reg_new.pc = this-> f_reg.pc;
    this-> d_reg_new.inst = aluCal;
    this-> d_reg_new.dest = destination;
    this-> d_reg_new.op1 = op1;
    this-> d_reg_new.op2 = op2;
    this-> d_reg_new.offset = offset;
}

/* initializeControlSignal()
 * Parameters: write_r, write_m, read_m, read_sign_ext, branch
 * Initializes control signals for the current instruction to default values.
 * All control signals are set to false indicating no operation or default behavior.
 * This function is called at the beginning of processing each instruction to reset the control signals.
 * write_r: Controls whether the instruction writes to a register.
 * write_m: Controls whether the instruction writes to memory.
 * read_m: Controls whether the instruction reads from memory.
 * read_sign_ext: Indicates if the instruction requires sign extension for its immediate value.
 * branch: Indicates if the instruction is a branch instruction.
 */
void Simulator::initializeControlSignal(bool &write_r, bool &write_m, bool &read_m, bool &read_sign_ext, bool &branch){
    write_r = write_m = read_m = read_sign_ext = branch = false;
}


/* execute()
 * Executes the decoded instruction by performing the appropriate ALU operation or control action.
 * Updates program counter for branch instructions and handles forwarding for data hazards.
 * Determines whether to write to registers or memory, and calculates the result of the ALU operation.
 * Also handles system call instructions (ECALL) and updates the execution stage register with the outcome.
 * Utilizes the control signals to guide the execution, including write enable for register and memory,
 * and read control for memory with or without sign extension. Determines branching and calculates
 * the next program counter if necessary.
 */
void Simulator::excecute() {
    // Handle stalls and bubbles
    if (this->d_reg.stall > 0 || this->d_reg.bubble) {
        this->e_reg_new.bubble = true;
        return;
    }
    this->history.inst_count++;

    // Extract instruction details
    Instruction aluCal = this->d_reg.inst;
    int32_t op1 = this->d_reg.op1;
    int32_t op2 = this->d_reg.op2;
    int32_t offset = this->d_reg.offset;
    uint32_t decodePC = this->d_reg.pc;
    int32_t result = 0;
    uint32_t mem_len = 0;
    uint32_t dest_reg = this->d_reg.dest;

    bool write_r, write_m, read_unsigned_m, read_sign_ext, branch;
    initializeControlSignal(write_r, write_m, read_unsigned_m, read_sign_ext, branch);

   // std::cout << "--aluCal--" << aluCal << std::endl;
    // Execute based on ALU operation
    // Includes operations for LUI, AUIPC, branch instructions, load/store, arithmetic, and logical instructions
    // For branch instructions, calculate the new PC
    // For load/store instructions, calculate the memory address
    // For arithmetic and logical instructions, calculate the result
    // System call handling for ECALL instructions
    if (aluCal == LUI) {
        write_r = true;
        result = offset << 12;
    } else if (aluCal == AUIPC) {
        write_r = true;
        result = decodePC + (offset << 12);
    } else if (aluCal == JAL) {
        write_r = true;
        result = decodePC + 4;
        decodePC = decodePC + op1;
        branch = true;
    } else if (aluCal == JALR) {
        write_r = true;
        result = decodePC + 4;
        decodePC = (op1 + op2) & (~(uint32_t) 1);
        branch = true;
    } else if (aluCal == BEQ) {
        if (op1 == op2) {
            branch = true;
            decodePC = decodePC + offset;
        }
    } else if (aluCal == BNE) {
        if (op1 != op2) {
            branch = true;
            decodePC = decodePC + offset;
        }
    } else if (aluCal == BLT) {
        if (op1 < op2) {
            branch = true;
            decodePC = decodePC + offset;
        }
    } else if (aluCal == BGE) {
        if (op1 >= op2) {
            branch = true;
            decodePC = decodePC + offset;
        }
    } else if (aluCal == BLTU) {
        if ((uint32_t)op1 < (uint32_t)op2) {
            branch = true;
            decodePC = decodePC + offset;
        }
    } else if (aluCal == BGEU) {
        if ((uint32_t) op1 >= (uint32_t) op2) {
            branch = true;
            decodePC = decodePC + offset;
        }
    } else if (aluCal == LB || aluCal == LH || aluCal == LW) {
        if (aluCal == LB) {
            mem_len = 1;
            result = op1 + offset;
            read_sign_ext = true;
            write_r = true;
        } else if (aluCal == LH) {
            mem_len = 2;
            result = op1 + offset;
            read_sign_ext = true;
            write_r = true;
        } else { // LW
            mem_len = 4;
            result = op1 + offset;
            read_sign_ext = true;
            write_r = true;
        }
    } else if (aluCal == LBU) {
        read_unsigned_m = true;
        write_r = true;
        mem_len = 1;
        result = op1 + offset;
    } else if (aluCal == LHU){
        read_unsigned_m = true;
        write_r = true;
        mem_len = 2;
        result = op1 + offset;
    } else if (aluCal == SB || aluCal == SH || aluCal == SW) {
        write_m = true;
        if (aluCal == SB) {
            mem_len = 1;
        } else if (aluCal == SH) {
            mem_len = 2;
        } else {
            mem_len = 4;
        }
        if (aluCal == SB) {
            result = op1 + offset;
            op2 = op2 & 0xFF;
        } else if (aluCal == SH) {
            result = op1 + offset;
            op2 = op2 & 0xFFFF;
        } else { // SW
            result = op1 + offset;
        }
    } else if (aluCal == ADDI) {
        write_r = true;
        result = op1 + op2;
    } else if (aluCal == SLTI) {
        write_r = true;
        result = (op1 < op2) ? 1 : 0;
    } else if (aluCal == SLTIU) {
        write_r = true;
        result = ((uint32_t) op1 < (uint32_t) op2) ? 1 : 0;
    } else if (aluCal == XORI) {
        write_r = true;
        result = op1 ^ op2;
    } else if (aluCal == ORI) {
        write_r = true;
        result = op1 | op2;
    } else if (aluCal == ANDI) {
        write_r = true;
        result = op1 & op2;
    } else if (aluCal == SLLI || aluCal == SRLI || aluCal == SRAI) {
        write_r = true;
        if (aluCal == SLLI) {
            result = op1 << op2;
        } else if (aluCal == SRLI) {
            result = (uint32_t)op1 >> (uint32_t)op2;
        } else if (aluCal == SRAI) {
            result = op1 >> op2;
        }
    } else if (aluCal == ADD) {
        write_r = true;
        result = op1 + op2;
    } else if (aluCal == SUB) {
        write_r = true;
        result = op1 - op2;
    } else if (aluCal == SLL) {
        write_r = true;
        result = op1 << op2;
    } else if (aluCal == SLT) {
        write_r = true;
        result = (op1 < op2) ? 1 : 0;
    } else if (aluCal == SLTU) {
        write_r = true;
        result = ((uint32_t) op1 < (uint32_t) op2) ? 1 : 0;
    } else if (aluCal == XOR) {
        write_r = true;
        result = op1 ^ op2;
    } else if (aluCal == SRL) {
        write_r = true;
        result = (uint32_t) op1 >> (uint32_t) op2;
    } else if (aluCal == SRA) {
        write_r = true;
        result = op1 >> op2;
    } else if (aluCal == OR) {
        write_r = true;
        result = op1 | op2;
    } else if (aluCal == AND) {
        write_r = true;
        result = op1 & op2;
    } else if (aluCal == ECALL) {
        result = handle_system_call(op1, op2);
        write_r = true;
    }

    // For branch instructions, the PC needs to be updated to the new address calculated during execution.
    // Insert bubbles into the pipeline to prevent the execution of instructions that should be cancelled by the taken branch.
    if (branch){
        control_hazard_update(decodePC);
        this ->history.control_hazard_count++;
    }

    if (aluCal == JAL || aluCal == JALR){
        control_hazard_update(decodePC);
        this ->history.control_hazard_count++;
    }

    // Data hazard
    bool isWriteNeeded = write_r && (dest_reg != 0);
    if (isWriteNeeded) {
        bool shouldForward = !(aluCal == LB || aluCal == LH || aluCal == LW || aluCal == LBU || aluCal == LHU);
        // Forward data to the first source operand of the next instruction (rs1)
        if (shouldForward && this ->d_reg_new.rs1 == dest_reg) {
            // Ensure not repeatedly forward to the same destination registers
            if (!this ->execute_write_back || (this ->execute_wb_reg != dest_reg)) {
                updateExecuteWB(execute_write_back, execute_wb_reg, dest_reg);
                this ->d_reg_new.op1 = result;
                this ->history.data_hazard_count++;
            }
        }
        if (shouldForward && this->d_reg_new.rs2 == dest_reg) {
            // Ensure not repeatedly forward to the same destination registers
            if (!this ->execute_write_back || (this ->execute_wb_reg != dest_reg)) {
                updateExecuteWB(execute_write_back, execute_wb_reg, dest_reg);
                this->d_reg_new.op2 = result;
                this->history.data_hazard_count++;
            }
        }
    }

    // Memory hazard
    // Control hazards also occur with jump instructions (JAL, JALR), requiring updates to the program counter.
    if (aluCal == LB || aluCal == LH || aluCal == LW || aluCal == LBU || aluCal == LHU){
        bool memoryRep = this ->d_reg_new.rs1 == dest_reg || this ->d_reg_new.rs2 == dest_reg;
        if (memoryRep){
            memory_hazard_update();
            this ->history.mem_hazard_count++;
        }
    }

    // Update execution stage register with the results of execution
    this ->e_reg_new.bubble = false;
    this ->e_reg_new.stall = false;
    this ->e_reg_new.pc = decodePC;
    this ->e_reg_new.inst = aluCal;
    this ->e_reg_new.op1 = op1;
    this ->e_reg_new.op2 = op2;
    this ->e_reg_new.write_reg = write_r;
    this ->e_reg_new.dest_reg = dest_reg;
    this ->e_reg_new.out = result;
    this ->e_reg_new.write_mem = write_m;
    this ->e_reg_new.read_unsigned_mem = read_unsigned_m;
    this ->e_reg_new.read_signed_mem = read_sign_ext;
    this ->e_reg_new.mem_len = mem_len;
    this ->e_reg_new.branch= branch;
}

/* control_hazard_update()
 * Updates the simulator state to handle a control hazard, typically arising from branch or jump instructions.
 * Parameters: decodePC: The updated program counter (PC) value calculated during execution to jump to.
 * Sets the main program counter (pc) to decodePC, effectively changing the execution flow to the new address.
 * Inserts bubbles into the fetch (f_reg_new) and decode (d_reg_new) pipeline stages.
 */
void Simulator::control_hazard_update(uint32_t decodePC){
    this ->pc = decodePC;
    this ->f_reg_new.bubble = this ->d_reg_new.bubble = true;
}

/* memory_hazard_update()
 * Since load operations may take more than one cycle to complete, subsequent instructions that rely on loaded data must wait.
 * Set stalls in the fetch (f_reg_new) and decode (d_reg_new) stages by setting their stall counters to 2.
 * Inserts a bubble into the execute (e_reg_new) stage, indicating that no instruction should be executed
 * Decrements the cycle count in the simulator's history, correcting for the artificial increase in cycle
 */
void Simulator::memory_hazard_update() {
    this ->f_reg_new.stall = this ->d_reg_new.stall = 2;
    this ->e_reg_new.bubble = true;
    //this ->history.cycle_count--;
}


/* memory_access()
 * Executes the memory access stage of the pipeline. This stage is responsible for reading from or writing to memory
 * based on the instruction type, such as load, store, or branch instructions that require memory interaction.
 * It also handles forwarding of results to bypass data hazards and manages stalls and bubbles in the pipeline.
 */
void Simulator::memory_access() {
    //Checks for stalls or bubbles in the execution register. If found, appropriately sets the next memory stage register to bubble or returns early to wait for the stall to resolve.
    if (this ->e_reg.stall > 0 || this ->e_reg.bubble) {
        if (this -> e_reg.stall > 0){
            return;
        } else{
            this->m_reg_new.bubble = true;
            return;
        }
    }

    //Reads or writes memory based on the instruction's requirements, updating the result with the memory operation outcome.
    uint32_t eRegPC = this ->e_reg.pc;
    Instruction aluCal = this ->e_reg.inst;
    bool write_reg = this ->e_reg.write_reg;
    uint32_t dest_reg = this ->e_reg.dest_reg;
    int32_t op1 = this ->e_reg.op1; // for jalr
    int32_t op2 = this ->e_reg.op2; // for store
    int32_t result = this ->e_reg.out;
    bool write_mem = this ->e_reg.write_mem;
    bool read_unsign_mem = this ->e_reg.read_unsigned_mem;
    bool read_sign = this ->e_reg.read_signed_mem;
    uint32_t mem_length = this ->e_reg.mem_len;

    bool memory_access_result;
    uint32_t cycle = 0;

    //Sign extension for signed memory read operations and corrects data types for unsigned reads.
    switch (read_unsign_mem) {
        case true:
            if (mem_length == 1) {
                result = (uint32_t) this->memory->get_byte(result, &cycle);
            } else if (mem_length == 2) {
                result = (uint32_t) this->memory->get_short(result, &cycle);
            } else if (mem_length == 4) {
                result = (uint32_t) this->memory->get_int(result, &cycle);
            }
    }

    switch (read_sign) {
        case true:
            if (mem_length == 1) {
                result = (int32_t) this->memory->get_byte(result, &cycle);
            } else if (mem_length == 2) {
                result = (int32_t) this->memory->get_short(result, &cycle);
            } else if (mem_length == 4) {
                result = (int32_t) this->memory->get_int(result, &cycle);
            }
    }

    switch (write_mem) {
        case true:
            if (mem_length == 1) {
                memory_access_result = this->memory->set_byte(result, op2, &cycle);
            } else if (mem_length == 2) {
                memory_access_result = this->memory->set_short(result, op2, &cycle);
            } else if (mem_length == 4) {
                //std::cout << result << std::endl;
                memory_access_result = this->memory->set_int(result, op2, &cycle);
            }
    }

   // Implements data forwarding to solve data hazards by checking if the decoded instruction's source registers
   // match the current instruction's destination register. If so, forwards the result to avoid stalls.
    bool isForwardNeeded = write_reg && (dest_reg != 0);

    if(isForwardNeeded){
        bool canForward = (this ->execute_write_back == false || this ->execute_wb_reg != dest_reg);
        if (this->d_reg_new.rs1 == dest_reg && canForward) {
            this->d_reg_new.op1 = result;
            updateMemoryWB(memory_write_back, memory_wb_reg, dest_reg);
            this->history.data_hazard_count++;
        }

        if (this->d_reg_new.rs2 == dest_reg && canForward) {
            this->d_reg_new.op2 = result;
            updateMemoryWB(memory_write_back, memory_wb_reg, dest_reg);
            this->history.data_hazard_count++;
        }

        // Corner case of forwarding mem load data to stalled decode reg
        if (this->d_reg.stall && (this->d_reg.rs1 == dest_reg || this->d_reg.rs2 == dest_reg)) {
            if (this->d_reg.rs1 == dest_reg) this->d_reg.op1 = result;
            if (this->d_reg.rs2 == dest_reg) this->d_reg.op2 = result;
            updateMemoryWB(memory_write_back, memory_wb_reg, dest_reg);
            this->history.data_hazard_count++;
        }
    }

    //Updates the next memory stage register with the outcome of this stage, preparing it for the write-back stage.
    this ->m_reg_new.bubble = false;
    this ->m_reg_new.stall = false;
    this ->m_reg_new.pc = eRegPC;
    this ->m_reg_new.inst = aluCal;
    this ->m_reg_new.op1 = op1;
    this ->m_reg_new.op2 = op2;
    this ->m_reg_new.dest_reg = dest_reg;
    this ->m_reg_new.write_reg = write_reg;
    this ->m_reg_new.out = result;
}

/* write_back()
 * The write-back stage of the pipeline. This stage is responsible for writing the results of executed instructions
 * back into the register file, completing the instruction cycle.
 * It also implements forwarding to ensure that any instructions in the decode stage that depend on the results being written back can immediately access the updated data.
 */
void Simulator::write_back() {
    if (this ->m_reg.stall != 0 || this ->m_reg.bubble){
        return;
    }
    if (!m_reg.write_reg || m_reg.dest_reg == 0){
        return;
    }
    if (this ->m_reg.write_reg && this ->m_reg.dest_reg != 0) {
        // Attempt to forward the result of the execution to the instructions in the decode phase that are waiting for this result.
        // For each source operand that requires this data (rs1 and rs2), check and perform data forwarding.
        forward_data_to_decode(this->m_reg.dest_reg, this->m_reg.out);
    }
    this ->reg[this ->m_reg.dest_reg] = this ->m_reg.out;
}

/* forward_data_to_decode()
 * Parameters: destReg, out
 * Handles the forwarding of data from the write-back stage to the decode stage for any instructions that require the data being written back.
 * This function is called during the write-back stage if an instruction has completed execution and its result is needed by instructions currently in the decode stage.
 */
void Simulator::forward_data_to_decode(uint32_t destReg, int32_t out) {
    bool canForward = canForwardData(destReg);
    if (canForward) {
        if (this ->d_reg_new.rs1 == destReg) {
            this ->d_reg_new.op1 = out;
            this ->history.data_hazard_count++;
        }
        if (this ->d_reg_new.rs2 == destReg) {
            this ->d_reg_new.op2 = out;
            this ->history.data_hazard_count++;
        }
    }
}

/* canForwardData()
 * Parameters: destReg
 * Evaluates whether it's possible to forward data from the write-back stage to the decode stage for a given destination register.
 * This determination is based on current pipeline conditions and whether any previous instructions have already forwarded data to the same destination register.
 */
bool Simulator::canForwardData(uint32_t destReg) {
    if ((!this->execute_write_back || this->execute_wb_reg != destReg) && (!this->memory_write_back || this->memory_wb_reg != destReg)) {
        return true;
    }
    return false;
}

/* handle_system_call
 * Handle system according to system call number in reg a7
 * exit program using exit(0)
 */
int32_t Simulator::handle_system_call(int32_t op1, int32_t op2) {
    int32_t call = op1;
    int32_t call_type = op2;
    if(call_type == 0){
        uint32_t address = call;
        std::string output;
        char character = static_cast<char>(this -> memory -> get_byte(address));
        while (character != '\0') {
            output += character;
            character = static_cast<char>(this -> memory -> get_byte(++address));
        }
        std::cout << output;
    } else if (call_type == 1){
        char output = (char)call;
        std::cout << output;
    } else if (call_type == 2){
        int32_t output = (int32_t)call;
        std::cout << output;
    } else if (call_type == 3){
        if(his_print) this ->print_history();
        exit(0);
    } else if (call_type == 4){
        std::cin >> (char*)(&op1);
        std::cin >> std::skipws;
    } else if (call_type == 5){
        scanf(" %d", &op1);
    }

    return op1;
}

/* print_history()
 * Print the history of all the counts.
 */
void Simulator::print_history() {
    std::cout << "---------History---------" << std::endl;
    std::cout << "Cycle count = " << history.cycle_count << std::endl;
    std::cout << "Inst count = " << history.inst_count << std::endl;
    std::cout << "Stall cycle count = " << history.stall_cycle_count << std::endl;
    std::cout << "Data hazard count = " << history.data_hazard_count << std::endl;
    std::cout << "Control hazard count = " << history.control_hazard_count << std::endl;
    std::cout << "Memory hazard count = " << history.mem_hazard_count << std::endl;
    std::cout << "----------Exit-----------" << std::endl;
}
