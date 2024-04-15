#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "simulator/memory_manager.h"
#include <cstdint>

const int REGNUM = 32;

enum Instruction {
  LUI = 0,
  AUIPC = 1,
  JAL = 2,
  JALR = 3,
  BEQ = 4,
  BNE = 5,
  BLT = 6,
  BGE = 7,
  BLTU = 8,
  BGEU = 9,
  LB = 10,
  LH = 11,
  LW = 12,
  LD = 13,
  LBU = 14,
  LHU = 15,
  SB = 16,
  SH = 17,
  SW = 18,
  SD = 19,
  ADDI = 20,
  SLTI = 21,
  SLTIU = 22,
  XORI = 23,
  ORI = 24,
  ANDI = 25,
  SLLI = 26,
  SRLI = 27,
  SRAI = 28,
  ADD = 29,
  SUB = 30,
  SLL = 31,
  SLT = 32,
  SLTU = 33,
  XOR = 34,
  SRL = 35,
  SRA = 36,
  OR = 37,
  AND = 38,
  ECALL = 39,
  UNKNOWN = -1,
};

// opcode field
const int OP_REG = 0x33;
const int OP_IMM = 0x13;
const int OP_LUI = 0x37;
const int OP_BRANCH = 0x63;
const int OP_STORE = 0x23;
const int OP_LOAD = 0x03;
const int OP_SYSTEM = 0x73;
const int OP_AUIPC = 0x17;
const int OP_JAL = 0x6F;
const int OP_JALR = 0x67;

class Simulator {
public:
  uint32_t pc;
  uint32_t reg[REGNUM];
  uint32_t stack_base;
  uint32_t max_stack_size;
  MemoryManager* memory;
  bool his_print = false;

  //decode
  void extract_R_inst(uint32_t &reg1, uint32_t &reg2, uint32_t func3, uint32_t func7, uint32_t rs1, uint32_t rs2, uint32_t rd, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &dest);
  void extract_I_imm(uint32_t &reg1, uint32_t func3, uint32_t func7, int32_t imm_i, uint32_t rs1, uint32_t rd, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &dest);
  void extract_I_load(uint32_t &reg1, uint32_t func3, int32_t imm_i, uint32_t rs1, uint32_t rd, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &offset, uint32_t &dest);
  void extract_I_LUI(uint32_t rd, int32_t imm, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &offset, uint32_t &dest);
  void extract_I_AUIPC(uint32_t rd, int32_t imm, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &offset, uint32_t &dest);
  void extract_S(uint32_t &reg1, uint32_t &reg2, uint32_t func3, uint32_t rs1, uint32_t rs2, int32_t imm_s, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &offset);
  void extract_B(uint32_t &reg1, uint32_t &reg2, uint32_t func3, uint32_t rs1, uint32_t rs2, int32_t imm_s, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &offset);
  void extract_ECALL(uint32_t &reg1, uint32_t &reg2, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &dest);
  void extract_jalr(uint32_t &reg1, int32_t imm_jalr, uint32_t rs1, uint32_t rd, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &dest);
  void extract_jal(uint32_t rd, int32_t imm_jal, Instruction &aluCal, uint32_t &op1, uint32_t &op2, uint32_t &offset, uint32_t &dest);

  Simulator(MemoryManager* memory);
  ~Simulator();

  void init_stack(uint32_t base_addr, uint32_t max_size);
  void simulate();
  bool pageNotExist(uint32_t addr);

  //execute
  void initializeControlSignal(bool &write_r, bool &write_m, bool &read_m, bool &read_sign_ext, bool &branch);
  void control_hazard_update(uint32_t dest_reg);
  void memory_hazard_update();

private:
  // can change according your need
  struct FetchRegister {
    bool bubble;
    uint32_t stall;

    uint32_t pc;
    uint32_t inst;
    uint32_t len;
      FetchRegister(const bool bubble = false, const uint32_t stall = 0,
                    const uint32_t pc = 0, const uint32_t inst = 0,
                    const uint32_t len = 0) : bubble(bubble), stall(stall),
                                              pc(pc), inst(inst), len(len) {}
  } f_reg, f_reg_new;

  // can change according your need
  struct DecodeRegister {
    bool bubble;
    uint32_t stall;
    uint32_t rs1, rs2;

    uint32_t pc;
    Instruction inst;
    int32_t op1;
    int32_t op2;
    uint32_t dest;
    int32_t offset;
      DecodeRegister(const bool bubble = false, const uint32_t stall = 0,
                     const uint32_t rs1 = 0, const uint32_t rs2 = 0,
                     const uint32_t pc = 0, const Instruction inst = LUI,
                     const int32_t op1 = 0, const int32_t op2 = 0,
                     const uint32_t dest = 0, const int32_t offset = 0) :
              bubble(bubble), stall(stall), rs1(rs1), rs2(rs2), pc(pc), inst(inst),
              op1(op1), op2(op2), dest(dest), offset(offset) {}
  } d_reg, d_reg_new;

  // can change according your need
  struct ExecuteRegister {
    bool bubble;
    uint32_t stall;

    uint32_t pc;
    Instruction inst;
    int32_t op1;
    int32_t op2;
    bool write_reg;
    uint32_t dest_reg;
    // output of execute stage
    int32_t out;
    bool write_mem;
    bool read_unsigned_mem;
    bool read_signed_mem;
    uint32_t mem_len;
    bool branch;
      ExecuteRegister(const bool bubble = false, const uint32_t stall = 0,
                      const uint32_t pc = 0, const Instruction inst = LUI,
                      const int32_t op1 = 0, const int32_t op2 = 0,
                      const bool write_reg = false, const uint32_t dest_reg = 0,
                      const int32_t out = 0, const bool write_mem = false,
                      const bool read_unsigned_mem = false,
                      const bool read_signed_mem = false,
                      const uint32_t mem_len = 0) :
              bubble(bubble), stall(stall), pc(pc), inst(inst), op1(op1), op2(op2),
              write_reg(write_reg), dest_reg(dest_reg), out(out),
              write_mem(write_mem), read_unsigned_mem(read_unsigned_mem),
              read_signed_mem(read_signed_mem), mem_len(mem_len) {}
  } e_reg, e_reg_new;

  // can change according your need
  struct MemoryRegister {
    bool bubble;
    uint32_t stall;

    uint32_t pc;
    Instruction inst;
    int32_t op1;
    int32_t op2;
    int32_t out;
    bool write_reg;
    uint32_t dest_reg;
      MemoryRegister(const bool bubble = false, const uint32_t stall = 0,
                     const uint32_t pc = 0, const Instruction inst = LUI,
                     const int32_t op1 = 0, const int32_t op2 = 0,
                     const int32_t out = 0, const bool write_reg = false,
                     const uint32_t dest_reg = 0) :
              bubble(bubble), stall(stall), pc(pc), inst(inst), op1(op1), op2(op2),
              out(out), write_reg(write_reg), dest_reg(dest_reg) {}
  } m_reg, m_reg_new;

  //
  bool execute_write_back;
  uint32_t execute_wb_reg;
  bool memory_write_back;
  uint32_t memory_wb_reg;

  void updateExecuteWB(bool &executeWriteBack, uint32_t &executeWBReg, uint32_t dest_reg){
      executeWriteBack = true;
      executeWBReg = dest_reg;
  }

  void updateMemoryWB(bool &memoryWriteBack, uint32_t &memoryWBReg, uint32_t dest_reg){
        memoryWriteBack = true;
        memoryWBReg = dest_reg;
  }

  bool canForwardData(uint32_t destReg);
  void forward_data_to_decode(uint32_t destReg, int32_t result);

  // can change according your need
  struct History {
    uint32_t inst_count;
    uint32_t cycle_count;
    uint32_t stall_cycle_count;

    uint32_t data_hazard_count;
    uint32_t control_hazard_count;
    uint32_t mem_hazard_count;
  } history;


  uint32_t readInstructionFromMemory(uint32_t *myPc);
  void updateFetchRegister(uint32_t inst, uint32_t length);

  void fetch();
  void decode();
  void excecute();
  void memory_access();
  void write_back();

  void print_history();
  int32_t handle_system_call(int32_t op1, int32_t op2);
};

#endif
