#![allow(dead_code)]
#![allow(overflowing_literals)]

use crate::memory::MemoryManager;

pub const WORD_BYTES: usize = 2;

#[derive(Clone, Copy)]
pub enum RegisterName {
    AccumulatorH,       // The results of operations (High: the first 8 bits, or the top/high-half)
    AccumulatorL,       // The results of operations (Low: the last 8 bits, or the bottom/low-half)
    StatusH,            // Arithmetic, logic, and control flags (High) (Flags below)
    StatusL,            // Arithmetic, logic, and control flags (Low) (Flags below)
    MemoryAddressWH,    // The high address being read from/written to in memory (High)
    MemoryAddressWL,    // The high address being read from/written to in memory (Low)
    MemoryAddressH,     // The low address being read from/written to in memory (High)
    MemoryAddressL,     // The low address being read from/written to in memory (Low)
    MemoryDataH,        // The data that is being read/written in memory (High)
    MemoryDataL,        // The data that is being read/written in memory (Low)
    ProgramCounterWH,   // The high address of the current place in execution (High)
    ProgramCounterWL,   // The high address of the current place in execution (Low)
    ProgramCounterH,    // The low address of the current place in execution (High)
    ProgramCounterL,    // The low address of the current place in execution (Low)
    InstructionH,       // The current data read from memory at the Program Counter (High)
    InstructionL,       // The current data read from memory at the Program Counter (Low)
    StackPointerWH,     // The high address of the current place in the stack (High)
    StackPointerWL,     // The high address of the current place in the stack (Low)
    StackPointerH,      // The low address of the current place in the stack (High)
    StackPointerL,      // The low address of the current place in the stack (Low)
    BasePointerWH,      // The high address of the base of the stack (High)
    BasePointerWL,      // The high address of the base of the stack (Low)
    BasePointerH,       // The low address of the base of the stack (High)
    BasePointerL,       // The low address of the base of the stack (Low)
    IndexAH,            // A high address pointer used for memory-related operations, primarily for addressing data (High)
    IndexAL,            // A high address pointer used for memory-related operations, primarily for addressing data (Low)
    IndexBH,            // A high address pointer used for memory-related operations, primarily for addressing code (High)
    IndexBL,            // A high address pointer used for memory-related operations, primarily for addressing code (Low)
    R0H,                // A general purpose register (High)
    R0L,                // A general purpose register (Low)
    R1H,                // A general purpose register (High)
    R1L,                // A general purpose register (Low)
    R2H,                // A general purpose register (High)
    R2L,                // A general purpose register (Low)
    R3H,                // A general purpose register (High)
    R3L,                // A general purpose register (Low)
} pub const REGISTERS: usize = 18;

// StatusH Flags: -.-.-.-.-.-.-.- 
// StatusL Flags: -.-.B.-.O.S.Z.C 
// C: Carry/Borrow,
// Z: Zero,
// S: Sign,
// O: Overflow,
// B: Byte Mode (Whether instructions work with; 0: 16-bit values, or 1: 8-bit values),
// -: Ignored

impl RegisterName {
    fn register_name_from_u8(num: u8) -> Option<RegisterName> {
        match num {
            0x00 => Some(RegisterName::AccumulatorH),
            0x01 => Some(RegisterName::IndexAH),
            0x02 => Some(RegisterName::IndexBH),
            0x03 => Some(RegisterName::R0H),
            0x04 => Some(RegisterName::R1H),
            0x05 => Some(RegisterName::R2H),
            0x06 => Some(RegisterName::R3H),

            0x80 => Some(RegisterName::AccumulatorL),
            0x81 => Some(RegisterName::IndexAL),
            0x82 => Some(RegisterName::IndexBL),
            0x83 => Some(RegisterName::R0L),
            0x84 => Some(RegisterName::R1L),
            0x85 => Some(RegisterName::R2L),
            0x86 => Some(RegisterName::R3L),
            _ => None,
        }
    }
}

enum ByteModeOverride {
    ForceOn,
    ForceOff,
    DontOverride,
}


pub struct Cpu {
    registers: MemoryManager,
    halt: bool,
}

impl Cpu {
    pub fn new() -> Cpu {
        let registers_size = REGISTERS * WORD_BYTES;
        Cpu {
            registers: MemoryManager::new(registers_size),
            halt: false,
        }
    }

    pub fn is_halted(&self) -> bool {
        self.halt
    }

    pub fn byte_mode_is_set(&self) -> bool {
        (self.read_register16(RegisterName::StatusH) & 0x0000_0020) != 0
    }


    pub fn read_register8(&self, reg: RegisterName) -> u8 {
        let reg_index = reg as usize;

        self.registers.read8(reg_index, false)
    }

    pub fn read_register16(&self, reg: RegisterName) -> u16 {
        let reg_index = reg as usize;

        self.registers.read16(reg_index, false)
    }

    pub fn read_register32(&self, reg: RegisterName) -> u32 {
        let reg_index = reg as usize;

        self.registers.read32(reg_index, false)
    }

    
    pub fn write_register8(&mut self, reg: RegisterName, value: u8) {
        let reg_index = reg as usize;

        self.registers.write8(reg_index, value, false);
    }

    pub fn write_register16(&mut self, reg: RegisterName, value: u16) {
        let reg_index = reg as usize;

        self.registers.write16(reg_index, value, false);
    }

    pub fn write_register32(&mut self, reg: RegisterName, value: u32) {
        let reg_index = reg as usize;

        self.registers.write32(reg_index, value, false);
    }


    pub fn copy_register8(&mut self, from: RegisterName, to: RegisterName) {
        self.write_register8(to, self.read_register8(from))
    }

    pub fn copy_register16(&mut self, from: RegisterName, to: RegisterName) {
        self.write_register16(to, self.read_register16(from))
    }

    pub fn copy_register32(&mut self, from: RegisterName, to: RegisterName) {
        self.write_register32(to, self.read_register32(from))
    }


    /// Set or clear bits in a 8-bit register
    /// 
    /// Takes a [RegisterName], a [u8] of the bits to be written, and a [bool] denoting whether the bits will be set or cleared

    pub fn register8_write_bits(&mut self, reg: RegisterName, bits: u8, set_or_clear: bool) {
        let value = self.read_register8(reg);

        if set_or_clear {
            self.write_register8(reg, value ^ bits);
        } else {
            self.write_register8(reg, value & !bits);
        }
    }

    /// Set or clear bits in a 16-bit register
    /// 
    /// Takes a [RegisterName], a [u16] of the bits to be written, and a [bool] denoting whether the bits will be set or cleared

    pub fn register16_write_bits(&mut self, reg: RegisterName, bits: u16, set_or_clear: bool) {
        let value = self.read_register16(reg);

        if set_or_clear {
            self.write_register16(reg, value ^ bits);
        } else {
            self.write_register16(reg, value & !bits);
        }
    }

    /// Set or clear bits in a 32-bit register
    /// 
    /// Takes a [RegisterName], a [u32] of the bits to be written, and a [bool] denoting whether the bits will be set or cleared

    pub fn register32_write_bits(&mut self, reg: RegisterName, bits: u32, set_or_clear: bool) {
        let value = self.read_register32(reg);

        if set_or_clear {
            self.write_register32(reg, value ^ bits);
        } else {
            self.write_register32(reg, value & !bits);
        }
    }


    /// Clock the CPU, performing one fetch-execute cycle
    /// 
    /// Takes a mutable reference to [Memory]

    pub fn clock(&mut self, mem: &mut MemoryManager) {
        self.fetch(mem);

        self.execute(mem);
    }


    /// Enable the data bus
    /// 
    /// Takes a mutable reference to [Memory], and,
    /// 
    /// If write_or_read = true, puts the 16-bit value from the MDR (MemoryDataRegister) into memory at the location pointed to by the 32-bit MAR (MemoryAddressRegister)
    /// 
    /// If write_or_read = false, puts the 16-bit value from memory pointed to by the 32-bit MAR into the MDR
    /// 
    /// Will fetch data based on the CPU's byte mode flag unless overridden
    
    fn memory_enable(&mut self, mem: &mut MemoryManager, write_or_read: bool, byte_mode_override: ByteModeOverride) {
        let mem_addr = self.read_register32(RegisterName::MemoryAddressWH);

        let byte_mode = match byte_mode_override {
            ByteModeOverride::ForceOn => true,
            ByteModeOverride::ForceOff => false,
            ByteModeOverride::DontOverride => self.byte_mode_is_set(),
        };

        if write_or_read {
            if !byte_mode {
                let mem_data = self.read_register16(RegisterName::MemoryDataH);

                mem.write16(mem_addr as usize, mem_data, false);
            } else {
                let mem_data = self.read_register8(RegisterName::MemoryDataL);

                mem.write8(mem_addr as usize, mem_data, false);
            }
        } else {
            if !byte_mode {
                let mem_data = mem.read16(mem_addr as usize, false);

                self.write_register16(RegisterName::MemoryDataH, mem_data);
            } else {
                let mem_data = mem.read8(mem_addr as usize, false);

                self.write_register8(RegisterName::MemoryDataL, mem_data);
            }
        }
    }
    
    /// Fetch next 2 bytes in memory pointed to by the Program Counter and store it in the Instruction register
    /// 
    /// Will always fetch 16-bit values regardless of the byte mode flag

    fn fetch(&mut self, mem: &mut MemoryManager) {
        // Copy the Program Counter to the Memory Address register
        self.copy_register32(RegisterName::ProgramCounterWH, RegisterName::MemoryAddressWH);
        
        // Enable memory to read to the Memory Data register
        self.memory_enable(mem, false, ByteModeOverride::ForceOff);

        // Copy the Memory Data register to the Instruction register
        self.copy_register16(RegisterName::MemoryDataH, RegisterName::InstructionH);
    }

    /// Advance the Program Counter past fetched bytes
     
    fn advance_pc(&mut self, amount: u32) {
        self.write_register32(RegisterName::ProgramCounterWH, self.read_register32(RegisterName::ProgramCounterWH) + amount);
    }

    /// Fetch register number then advance the Program Counter forward past the operand, return a RegisterName according to byte mode
    /// 
    /// Will return a RegisterName according to byte mode unless overridden

    fn fetch_register_operand(&mut self, mem: &mut MemoryManager, byte_mode_override: ByteModeOverride) -> Option<RegisterName> {
        self.fetch(mem);
        self.advance_pc(1);

        let byte_mode = match byte_mode_override {
            ByteModeOverride::ForceOn => true,
            ByteModeOverride::ForceOff => false,
            ByteModeOverride::DontOverride => self.byte_mode_is_set(),
        };

        if !byte_mode {
            RegisterName::register_name_from_u8(self.read_register8(RegisterName::InstructionH) & 0x7F)
        } else {
            RegisterName::register_name_from_u8(self.read_register8(RegisterName::InstructionH))
        }
    }

    /// Fetch a byte then advance the Program Counter forward past the operand, return the value

    fn fetch_operand8(&mut self, mem: &mut MemoryManager) -> u8 {
        self.fetch(mem);
        self.advance_pc(1);

        self.read_register8(RegisterName::InstructionH)
    }

    /// Fetch a word then advance the Program Counter forward past the operand, return the value

    fn fetch_operand16(&mut self, mem: &mut MemoryManager) -> u16 {
        self.fetch(mem);
        self.advance_pc(2);

        self.read_register16(RegisterName::InstructionH)
    }

    // Push a word to the stack

    fn push16(&mut self, mem: &mut MemoryManager, value: u16) {
        self.copy_register32(RegisterName::StackPointerWH, RegisterName::MemoryAddressWH);

        self.write_register16(RegisterName::MemoryDataH, value);

        self.memory_enable(mem, true, ByteModeOverride::DontOverride);

        self.write_register32(RegisterName::StackPointerWH, self.read_register32(RegisterName::StackPointerWH) - 2);
    }

    // Push a Byte to the stack

    fn push8(&mut self, mem: &mut MemoryManager, value: u8) {
        self.copy_register32(RegisterName::StackPointerWH, RegisterName::MemoryAddressWH);

        self.write_register8(RegisterName::MemoryDataL, value);

        self.memory_enable(mem, true, ByteModeOverride::DontOverride);

        self.write_register32(RegisterName::StackPointerWH, self.read_register32(RegisterName::StackPointerWH) - 1);
    }

    // Pop a word from the stack

    fn pop16(&mut self, mem: &mut MemoryManager) -> u16{
        self.write_register32(RegisterName::StackPointerWH, self.read_register32(RegisterName::StackPointerWH) + 2);

        self.copy_register32(RegisterName::StackPointerWH, RegisterName::MemoryAddressWH);

        self.memory_enable(mem, false, ByteModeOverride::DontOverride);

        self.read_register16(RegisterName::MemoryDataH)
    }

    // Pop a Byte from the stack

    fn pop8(&mut self, mem: &mut MemoryManager) -> u8 {
        self.write_register32(RegisterName::StackPointerWH, self.read_register32(RegisterName::StackPointerWH) + 1);

        self.copy_register32(RegisterName::StackPointerWH, RegisterName::MemoryAddressWH);

        self.memory_enable(mem, false, ByteModeOverride::DontOverride);

        self.read_register8(RegisterName::MemoryDataL)
    }

    /// Decode the first byte in the Instruction register and call its function

    fn execute(&mut self, mem: &mut MemoryManager) {
        let opcode = self.read_register8(RegisterName::InstructionH);

        // Advance the Program Counter forward by 1 byte, pointing past the opcode
        self.advance_pc(1);

        match opcode {
            0x01 => self.inst_ldi(mem),     // LoaD Immediate
            0x02 => self.inst_ldr(mem),     // LoaD Register
            0x03 => self.inst_ldm(mem),     // LoaD Memory
            0x04 => self.inst_ldp(mem),     // LoaD Pointer
            0x05 => self.inst_stai(mem),    // STore at Address an Immediate
            0x06 => self.inst_star(mem),    // STore at Address a Register
            0x07 => self.inst_stpi(mem),    // STore at Pointer an Immediate
            0x08 => self.inst_stpr(mem),    // STore at Pointer a Register
            0x09 => self.inst_swp(mem),     // SWaP registers

            0x0A => self.inst_inist(mem),   // INItialize STack to an address
            0x0B => self.inst_pshi(mem),    // PuSH Immediate
            0x0C => self.inst_pshr(mem),    // PuSH Register
            0x0D => self.inst_pop(mem),     // POP
            0x0E => self.inst_ldbo(mem),    // LoaD from Base pointer Offset
            0x0F => self.inst_stbo(mem),    // STore to Base pointer Offset
            
            0x10 => self.inst_add(mem),     // ADD registers
            0x11 => self.inst_addi(mem),    // ADD to register an Immediate
            0x12 => self.inst_sub(mem),     // SUBtract registers
            0x13 => self.inst_subi(mem),    // SUBtract from register an Immediate
            0x14 => self.inst_mul(mem),     // unsigned MULtiply registers
            0x15 => self.inst_smul(mem),    // Signed MULtiply registers
            0x16 => self.inst_div(mem),     // unsigned DIVide registers
            0x17 => self.inst_sdiv(mem),    // Signed DIVide registers
            0x18 => self.inst_inc(mem),     // INCrement register
            0x19 => self.inst_dec(mem),     // DECrement register
            0x1A => self.inst_lsh(mem),     // Left-SHift register
            0x1B => self.inst_rsh(mem),     // Right-SHift register
            0x1C => self.inst_and(mem),     // bit-AND registers
            0x1D => self.inst_or(mem),      // bit-OR registers
            0x1E => self.inst_xor(mem),     // bit-XOR registers
            0x1F => self.inst_not(mem),     // bit-NOT register
            0x20 => self.inst_stb(mem),     // SeT Bits in register
            0x21 => self.inst_clb(mem),     // CLear Bits in register
            0x22 => self.inst_tsb(mem),     // TeSt Bits in register

            0x30 => self.inst_jmp(mem),     // JuMP to address
            0x31 => self.inst_jmpr(mem),    // JuMP Relative
            0x32 => self.inst_jmpp(mem),    // JuMP to Pointer
            0x33 => self.inst_cmpi(mem),    // CoMPare register to Immediate
            0x34 => self.inst_cmpr(mem),    // CoMPare Registers
            0x35 => self.inst_je(mem),      // Jump relative if Zero / Jump relative if Equal
            0x36 => self.inst_jne(mem),     // Jump relative if Not Zero / Jump relative if Not Equal
            0x37 => self.inst_jl(mem),      // Jump relative if Sign / Jump relative if Lesser
            0x38 => self.inst_jg(mem),      // Jump relative if Greater
            0x39 => self.inst_jle(mem),     // Jump relative if Lesser or Equal
            0x3A => self.inst_jge(mem),     // Jump relative if Not Sign / Jump relative if Greater or Equal
            0x3B => self.inst_jc(mem),      // Jump relative if Carry
            0x3C => self.inst_jnc(mem),     // Jump relative if Not Carry
            0x3D => self.inst_jo(mem),      // Jump relative if Overflow
            0x3E => self.inst_jno(mem),     // Jump relative if Not Overflow

            0x40 => self.inst_cla(mem),     // CalL subroutine at Address
            0x41 => self.inst_clp(mem),     // CalL subroutine at Pointer
            0x42 => self.inst_ret(mem),     // RETurn from subroutine

            0xF0 => self.inst_sbm(),        // Set Byte Mode
            0xF1 => self.inst_cbm(),        // Clear Byte Mode
            0xFF => self.inst_hlt(),        // HaLT
            _ => return,                    // No-OPeration
        }
    }


    // Instructions:

    /// Load an immediate value to a register
    /// 
    /// All load/store instructions prefix thin addresses with IndexA

    fn inst_ldi(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3-4 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The next 1-2 byte(s) are an immediate value
        // If not in byte mode, fetch a word, else fetch a byte (it doesn't store the return value here since it will get the value from the instruction register later when it's more convenient)
        if !self.byte_mode_is_set() {
            self.fetch_operand16(mem);
        } else {
            self.fetch_operand8(mem);
        }

        // Shadow the register to be a RegisterName, it doesn't do this immediately to avoid returning before moving the Program Counter past the instruction
        // If the operand in the instruction is not a valid register number, it returns, since, in a physical machine, the state wouldn't change
        let reg = match reg {
            Some(x) => x,
            None => return,
        };

        if !self.byte_mode_is_set() {
            // The immediate value fetched was put into the Instruction register, copy the instruction register to the register
            self.copy_register16(RegisterName::InstructionH, reg);
        } else {
            // Copy 8-bit value instead of 16-bit, the immediate value is still in the top half of the instruction register since fetching always reads 16-bit values to the Instruction register
            self.copy_register8(RegisterName::InstructionH, reg);
        }
    }

    /// Load a register to a register

    fn inst_ldr(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second byte is a register
        let to_reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);
        
        // The third byte is a register
        let from_reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let to_reg = match to_reg {
            Some(x) => x,
            None => return,
        };

        let from_reg = match from_reg {
            Some(x) => x,
            None => return,
        };

        if !self.byte_mode_is_set() {
            self.copy_register16(from_reg, to_reg);
        } else {
            self.copy_register8(from_reg, to_reg);
        }
    }

    /// Load a value from memory to a register

    fn inst_ldm(&mut self, mem: &mut MemoryManager) {
        // This instruction is 4 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The third and fourth bytes are a thin memory address
        self.fetch_operand16(mem);

        let reg = match reg {
            Some(x) => x,
            None => return,
        };

        // The address fetched is thin, meaning it's only the low 2 bytes of a wide memory address, the high 2 bytes are represented with the Index A register
        self.copy_register16(RegisterName::IndexAH, RegisterName::MemoryAddressWH);
        self.copy_register16(RegisterName::InstructionH, RegisterName::MemoryAddressH);

        // Enable memory in read mode
        self.memory_enable(mem, false, ByteModeOverride::DontOverride);

        if !self.byte_mode_is_set() {
            // Copy the value from the Memory Data register, where memory data gets read to/written from, to the register
            self.copy_register16(RegisterName::MemoryDataH, reg);
        } else {
            // Note that, in byte mode, it copies the lower-half of Memory Data to the register, since it enabled memory without overriding byte mode, meaning it read an 8-bit value to the Memory Data register, instead of fetching to the Instruction register, which always reads 16-bit values
            self.copy_register8(RegisterName::MemoryDataL, reg);
        }
    }

    /// Load a value from memory at a pointer to a register

    fn inst_ldp(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The third byte is a pointer, a 16-bit-only register which points to a memory address
        let pointer_reg = self.fetch_register_operand(mem, ByteModeOverride::ForceOff);

        let reg = match reg {
            Some(x) => x,
            None => return,
        };

        let pointer_reg = match pointer_reg {
            Some(x) => x,
            None => return,
        };

        // Registers can only hold 16-bit values, so they inherently can only contain a thin address as a pointer
        self.copy_register16(RegisterName::IndexAH, RegisterName::MemoryAddressWH);
        self.copy_register16(pointer_reg, RegisterName::MemoryAddressH);

        self.memory_enable(mem, false, ByteModeOverride::DontOverride);

        if !self.byte_mode_is_set() {
            self.copy_register16(RegisterName::MemoryDataH, reg);
        } else {
            self.copy_register8(RegisterName::MemoryDataL, reg);
        }
    }

    /// Store an immediate value to memory

    fn inst_stai(&mut self, mem: &mut MemoryManager) {
        // This instruction is 4-5 bytes long, the first byte is the opcode
        // The second and third bytes are a memory address
        let address = self.fetch_operand16(mem);
        
        // The next 1-2 byte(s) are an immediate value
        if !self.byte_mode_is_set() {
            self.fetch_operand16(mem);

            // Copy the 16-bit value from the Instruction register to the Memory Data register
            self.copy_register16(RegisterName::InstructionH, RegisterName::MemoryDataH);
        } else {
            self.fetch_operand8(mem);
            
            // Copy the 8-bit value from (the top-half of) the Instruction register to (the bottom-half of) the Memory Data register
            self.copy_register8(RegisterName::InstructionH, RegisterName::MemoryDataL);
        }

        self.copy_register16(RegisterName::IndexAH, RegisterName::MemoryAddressWH);
        self.write_register16(RegisterName::MemoryAddressH, address);

        // Enable memory in write mode
        self.memory_enable(mem, true, ByteModeOverride::DontOverride);
    }

    /// Store a register to memory

    fn inst_star(&mut self, mem: &mut MemoryManager) {
        // This instruction is 4 bytes long, the first byte is the opcode
        // The second and third bytes are a memory address
        let address = self.fetch_operand16(mem);
        
        // The third byte is a register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let reg = match reg {
            Some(x) => x,
            None => return,
        };
        
        if !self.byte_mode_is_set() {
            self.copy_register16(reg, RegisterName::MemoryDataH);
        } else {
            self.copy_register8(reg, RegisterName::MemoryDataL);
        }

        self.copy_register16(RegisterName::IndexAH, RegisterName::MemoryAddressWH);
        self.write_register16(RegisterName::MemoryAddressH, address);
        
        self.memory_enable(mem, true, ByteModeOverride::DontOverride);
    }

    /// Store an immediate value to memory at a pointer

    fn inst_stpi(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3-4 bytes long, the first byte is the opcode
        // The second byte is a pointer
        let pointer_reg = self.fetch_register_operand(mem, ByteModeOverride::ForceOff);

        // The next 1-2 byte(s) are an immediate value
        if !self.byte_mode_is_set() {
            self.fetch_operand16(mem);
        } else {
            self.fetch_operand8(mem);
        }

        let pointer_reg = match pointer_reg {
            Some(x) => x,
            None => return,
        };

        if !self.byte_mode_is_set() {
            self.copy_register16(RegisterName::InstructionH, RegisterName::MemoryDataH);
        } else {
            self.copy_register8(RegisterName::InstructionH, RegisterName::MemoryDataL);
        }
        
        self.copy_register16(RegisterName::IndexAH, RegisterName::MemoryAddressWH);
        self.copy_register16(pointer_reg, RegisterName::MemoryAddressH);
        
        // Enable memory in write mode
        self.memory_enable(mem, true, ByteModeOverride::DontOverride);
    }

    /// Store a register to memory at a pointer

    fn inst_stpr(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second byte is a pointer
        let pointer_reg = self.fetch_register_operand(mem, ByteModeOverride::ForceOff);
        
        // The third byte is a register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let pointer_reg = match pointer_reg {
            Some(x) => x,
            None => return,
        };

        let reg = match reg {
            Some(x) => x,
            None => return,
        };
        
        if !self.byte_mode_is_set() {
            self.copy_register16(reg, RegisterName::MemoryDataH);
        } else {
            self.copy_register8(reg, RegisterName::MemoryDataL);
        }
        
        self.copy_register16(RegisterName::IndexAH, RegisterName::MemoryAddressWH);
        self.copy_register16(pointer_reg, RegisterName::MemoryAddressH);

        self.memory_enable(mem, true, ByteModeOverride::DontOverride);
    }

    /// Swap two registers

    fn inst_swp(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg_a = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The third byte is a right register
        let reg_b = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let reg_a = match reg_a {
            Some(x) => x,
            None => return,
        };

        let reg_b = match reg_b {
            Some(x) => x,
            None => return,
        };

        if !self.byte_mode_is_set() {
            // Save the value of reg_b
            let b = self.read_register16(reg_b);

            self.copy_register16(reg_a, reg_b);

            // Write the saved b value to reg_a
            self.write_register16(reg_a, b)
        } else {
            let b = self.read_register8(reg_b);

            self.copy_register8(reg_a, reg_b);

            self.write_register8(reg_a, b)
        }
    }

    /// Initalize the stack
    /// 
    /// All stack instructions prefix thin addresses with IndexA

    fn inst_inist(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second and third bytes are an address
        self.fetch_operand16(mem);

        self.copy_register16(RegisterName::IndexAH, RegisterName::BasePointerWH);
        self.copy_register16(RegisterName::InstructionH, RegisterName::BasePointerH);

        self.copy_register32(RegisterName::BasePointerWH, RegisterName::StackPointerWH);
    }
    
    /// Push an immediate value to the stack

    fn inst_pshi(&mut self, mem: &mut MemoryManager) {
        // This instruction is 2-3 bytes long, the first byte is the opcode
        // The next 1-2 byte(s) are an immediate value
        if !self.byte_mode_is_set() {
            let value = self.fetch_operand16(mem);

            self.push16(mem, value);
        } else {
            let value = self.fetch_operand8(mem);

            self.push8(mem, value);
        }
    }

    /// Push register value to the stack

    fn inst_pshr(&mut self, mem: &mut MemoryManager) {
        // This instruction is 2 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let reg = match reg {
            Some(x) => x,
            None => return,
        };

        if !self.byte_mode_is_set() {
            self.push16(mem, self.read_register16(reg));
        } else {
            self.push8(mem, self.read_register8(reg));
        }
    }

    /// Pop value from stack into register

    fn inst_pop(&mut self, mem: &mut MemoryManager) {
        // This instruction is 2 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let reg = match reg {
            Some(x) => x,
            None => return,
        };

        if !self.byte_mode_is_set() {
            self.pop16(mem);

            self.copy_register16(RegisterName::MemoryDataH, reg);
        } else {
            self.pop8(mem);

            self.copy_register8(RegisterName::MemoryDataL, reg);
        }
    }

    /// Load from memory at an address relative to the Base Pointer

    fn inst_ldbo(&mut self, mem: &mut MemoryManager) {
        // This instruction is 4 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The third and fourth bytes are a thin address
        self.fetch_operand16(mem);

        let reg = match reg {
            Some(x) => x,
            None => return,
        };

        let offset = i16::from_be_bytes(self.read_register16(RegisterName::InstructionH).to_be_bytes()) as i32;

        let address = (i32::from_be_bytes(self.read_register32(RegisterName::BasePointerWH).to_be_bytes()) + offset) as u32;

        self.write_register32(RegisterName::MemoryAddressWH, address);

        self.memory_enable(mem, false, ByteModeOverride::DontOverride);

        if !self.byte_mode_is_set() {
            self.copy_register16(RegisterName::MemoryDataH, reg);
        } else {
            self.copy_register8(RegisterName::MemoryDataL, reg);
        }
    }

    /// Store to memory at an address relative to the Base Pointer

    fn inst_stbo(&mut self, mem: &mut MemoryManager) {
        // This instruction is 4 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The third and fourth bytes are a thin address
        self.fetch_operand16(mem);

        let reg = match reg {
            Some(x) => x,
            None => return,
        };

        let offset = i16::from_be_bytes(self.read_register16(RegisterName::InstructionH).to_be_bytes()) as i32;

        let address = (i32::from_be_bytes(self.read_register32(RegisterName::BasePointerWH).to_be_bytes()) + offset) as u32;

        self.write_register32(RegisterName::MemoryAddressWH, address);

        if !self.byte_mode_is_set() {
            self.copy_register16(reg, RegisterName::MemoryDataH);
        } else {
            self.copy_register8(reg, RegisterName::MemoryDataL);
        }

        self.memory_enable(mem, true, ByteModeOverride::DontOverride);
    }

    /// Add two registers and store the result in the Accumulator

    fn inst_add(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode,
        // The second byte is a register
        let reg_a = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The third byte is a register
        let reg_b = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let reg_a = match reg_a {
            Some(x) => x,
            None => return,
        };

        let reg_b = match reg_b {
            Some(x) => x,
            None => return,
        };

        let flags = if !self.byte_mode_is_set() {
            let a = self.read_register16(reg_a);
            let b = self.read_register16(reg_b);

            let signed_a = i16::from_be_bytes(a.to_be_bytes());
            let signed_b = i16::from_be_bytes(b.to_be_bytes());

            let sum = a.overflowing_add(b);

            self.write_register16(RegisterName::AccumulatorH, sum.0);

            (sum.1, sum.0 == 0, (sum.0 & 0x8000) != 0, signed_a.overflowing_add(signed_b).1)
        } else {
            let a = self.read_register8(reg_a);
            let b = self.read_register8(reg_b);

            let signed_a = i8::from_be_bytes(a.to_be_bytes());
            let signed_b = i8::from_be_bytes(b.to_be_bytes());

            let sum = a.overflowing_add(b);

            self.write_register8(RegisterName::AccumulatorL, sum.0);

            (sum.1, sum.0 == 0, (sum.0 & 0x80) != 0, signed_a.overflowing_add(signed_b).1)
        };

        // Carry/Borrow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0001, flags.0);
        // Zero flag
        self.register16_write_bits(RegisterName::StatusH, 0x0002, flags.1);
        // Sign flag
        self.register16_write_bits(RegisterName::StatusH, 0x0004, flags.2);
        // Overflow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0008, flags.3);
    }

    /// Add an immediate value to a register and store the result in the Accumulator

    fn inst_addi(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3-4 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The next 1-2 byte(s) are an immediate value
        if !self.byte_mode_is_set() {
            self.fetch_operand16(mem);
        } else {
            self.fetch_operand8(mem);
        }

        let reg = match reg {
            Some(x) => x,
            None => return,
        };

        let flags = if !self.byte_mode_is_set() {
            let a = self.read_register16(reg);
            let b = self.read_register16(RegisterName::InstructionH);

            let signed_a = i16::from_be_bytes(a.to_be_bytes());
            let signed_b = i16::from_be_bytes(b.to_be_bytes());

            let sum = a.overflowing_add(b);

            self.write_register16(RegisterName::AccumulatorH, sum.0);

            (sum.1, sum.0 == 0, (sum.0 & 0x8000) != 0, signed_a.overflowing_add(signed_b).1)
        } else {
            let a = self.read_register8(reg);
            let b = self.read_register8(RegisterName::InstructionH);

            let signed_a = i8::from_be_bytes(a.to_be_bytes());
            let signed_b = i8::from_be_bytes(b.to_be_bytes());

            let sum = a.overflowing_add(b);

            self.write_register8(RegisterName::AccumulatorL, sum.0);

            (sum.1, sum.0 == 0, (sum.0 & 0x80) != 0, signed_a.overflowing_add(signed_b).1)
        };

        // Carry/Borrow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0001, flags.0);
        // Zero flag
        self.register16_write_bits(RegisterName::StatusH, 0x0002, flags.1);
        // Sign flag
        self.register16_write_bits(RegisterName::StatusH, 0x0004, flags.2);
        // Overflow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0008, flags.3);
    }

    /// Subtract two registers and store the result in the Accumulator

    fn inst_sub(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg_a = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The third byte is a register
        let reg_b = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let reg_a = match reg_a {
            Some(x) => x,
            None => return,
        };

        let reg_b = match reg_b {
            Some(x) => x,
            None => return,
        };

        let flags = if !self.byte_mode_is_set() {
            let a = self.read_register16(reg_a);
            let b = self.read_register16(reg_b);
            let signed_a = i16::from_be_bytes(a.to_be_bytes());
            let signed_b = i16::from_be_bytes(b.to_be_bytes());

            let difference = a.overflowing_sub(b);

            self.write_register16(RegisterName::AccumulatorH, difference.0);

            (difference.1, difference.0 == 0, (difference.0 & 0x8000) != 0, signed_a.overflowing_sub(signed_b).1)
        } else {
            let a = self.read_register8(reg_a);
            let b = self.read_register8(reg_b);
            let signed_a = i8::from_be_bytes(a.to_be_bytes());
            let signed_b = i8::from_be_bytes(b.to_be_bytes());

            let difference = a.overflowing_sub(b);

            self.write_register8(RegisterName::AccumulatorL, difference.0);

            (difference.1, difference.0 == 0, (difference.0 & 0x80) != 0, signed_a.overflowing_sub(signed_b).1)
        };

        // Carry/Borrow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0001, flags.0);
        // Zero flag
        self.register16_write_bits(RegisterName::StatusH, 0x0002, flags.1);
        // Sign flag
        self.register16_write_bits(RegisterName::StatusH, 0x0004, flags.2);
        // Overflow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0008, flags.3);
    }

    /// Subtract an immediate value from a register and store the result in the Accumulator

    fn inst_subi(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3-4 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The next 1-2 byte(s) are an immediate value
        if !self.byte_mode_is_set() {
            self.fetch_operand16(mem);
        } else {
            self.fetch_operand8(mem);
        }

        let reg = match reg {
            Some(x) => x,
            None => return,
        };

        let flags = if !self.byte_mode_is_set() {
            let a = self.read_register16(reg);
            let b = self.read_register16(RegisterName::InstructionH);
            let signed_a = i16::from_be_bytes(a.to_be_bytes());
            let signed_b = i16::from_be_bytes(b.to_be_bytes());

            let difference = a.overflowing_sub(b);

            self.write_register16(RegisterName::AccumulatorH, difference.0);

            (difference.1, difference.0 == 0, (difference.0 & 0x8000) != 0, signed_a.overflowing_sub(signed_b).1)
        } else {
            let a = self.read_register8(reg);
            let b = self.read_register8(RegisterName::InstructionH);
            let signed_a = i8::from_be_bytes(a.to_be_bytes());
            let signed_b = i8::from_be_bytes(b.to_be_bytes());

            let difference = a.overflowing_sub(b);

            self.write_register8(RegisterName::AccumulatorL, difference.0);

            (difference.1, difference.0 == 0, (difference.0 & 0x80) != 0, signed_a.overflowing_sub(signed_b).1)
        };

        // Carry/Borrow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0001, flags.0);
        // Zero flag
        self.register16_write_bits(RegisterName::StatusH, 0x0002, flags.1);
        // Sign flag
        self.register16_write_bits(RegisterName::StatusH, 0x0004, flags.2);
        // Overflow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0008, flags.3);
    }

    /// Unsigned multiply two registers and store the result in the Accumulator

    fn inst_mul(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg_a = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The third byte is a register
        let reg_b = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let reg_a = match reg_a {
            Some(x) => x,
            None => return,
        };

        let reg_b = match reg_b {
            Some(x) => x,
            None => return,
        };

        let flags = if !self.byte_mode_is_set() {
            let a = self.read_register16(reg_b);
            let b = self.read_register16(reg_b);

            let product = a.overflowing_mul(b);

            self.write_register16(RegisterName::AccumulatorH, product.0);

            (product.0 == 0, (product.0 & 0x8000) != 0, product.1)
        } else {
            let a = self.read_register8(reg_a) as u16;
            let b = self.read_register8(reg_b) as u16;

            let product = a.overflowing_mul(b);

            self.write_register16(RegisterName::AccumulatorH, product.0);

            (product.0 == 0, (product.0 & 0x80) != 0, product.0 > u8::MAX as u16)
        };

        // Carry/Borrow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0001, flags.2);
        // Zero flag
        self.register16_write_bits(RegisterName::StatusH, 0x0002, flags.0);
        // Sign flag
        self.register16_write_bits(RegisterName::StatusH, 0x0004, flags.1);
        // Overflow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0008, flags.2);
    }

    /// Signed multiply two registers and store the result in the Accumulator

    fn inst_smul(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg_a = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The third byte is a register
        let reg_b = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let reg_a = match reg_a {
            Some(x) => x,
            None => return,
        };

        let reg_b = match reg_b {
            Some(x) => x,
            None => return,
        };

        // If the byte mode flag is 0, 16-bit values, else, 8-bit values
        let flags = if !self.byte_mode_is_set() {
            let a = i16::from_be_bytes(self.read_register16(reg_a).to_be_bytes());
            let b = i16::from_be_bytes(self.read_register16(reg_b).to_be_bytes());

            let product = a.overflowing_mul(b);

            self.write_register16(RegisterName::AccumulatorH, u16::from_be_bytes(product.0.to_be_bytes()));

            (product.0 == 0, (product.0 & 0x8000_i16) != 0, product.1)
        } else {
            let a = i8::from_be_bytes(self.read_register8(reg_a).to_be_bytes()) as i16;
            let b = i8::from_be_bytes(self.read_register8(reg_b).to_be_bytes()) as i16;

            let product = a.overflowing_mul(b);

            self.write_register16(RegisterName::AccumulatorH, u16::from_be_bytes(product.0.to_be_bytes()));

            (product.0 == 0, (product.0 & 0x0080_i16) != 0, product.1)
        };

        // Carry/Borrow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0001, flags.2);
        // Zero flag
        self.register16_write_bits(RegisterName::StatusH, 0x0002, flags.0);
        // Sign flag
        self.register16_write_bits(RegisterName::StatusH, 0x0004, flags.1);
        // Overflow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0008, flags.2);
    }

    /// Unsigned divide two registers and store the result in the Accumulator

    fn inst_div(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg_a = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The third byte is the right register to divide:
        let reg_b = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let reg_a = match reg_a {
            Some(x) => x,
            None => return,
        };

        let reg_b = match reg_b {
            Some(x) => x,
            None => return,
        };

        let flags = if !self.byte_mode_is_set() {
            let a = self.read_register16(reg_a);
            let b = self.read_register16(reg_b);

            let divisor = a / b;

            self.write_register16(RegisterName::AccumulatorH, divisor);

            (divisor == 0, (divisor & 0x8000) != 0)
        } else {
            let a = self.read_register8(reg_a);
            let b = self.read_register8(reg_b);

            let divisor = a / b;

            self.write_register8(RegisterName::AccumulatorL, divisor);

            (divisor == 0, (divisor & 0x80) != 0)
        };

        // Carry/Borrow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0001, false);
        // Zero flag
        self.register16_write_bits(RegisterName::StatusH, 0x0002, flags.0);
        // Sign flag
        self.register16_write_bits(RegisterName::StatusH, 0x0004, flags.1);
        // Overflow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0008, false);
    }

    /// Signed divide two registers and store the result in the Accumulator

    fn inst_sdiv(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg_a = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The third byte is a register
        let reg_b = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let reg_a = match reg_a {
            Some(x) => x,
            None => return,
        };

        let reg_b = match reg_b {
            Some(x) => x,
            None => return,
        };

        let flags = if !self.byte_mode_is_set() {
            let a = i16::from_be_bytes(self.read_register16(reg_a).to_be_bytes());
            let b = i16::from_be_bytes(self.read_register16(reg_b).to_be_bytes());

            let divisor = a.overflowing_div(b);

            self.write_register16(RegisterName::AccumulatorH, u16::from_be_bytes(divisor.0.to_be_bytes()));

            (divisor.0 == 0, (divisor.0 & 0x8000_i16) != 0, divisor.1)
        } else {
            let a = i8::from_be_bytes(self.read_register8(reg_a).to_be_bytes()) as i16;
            let b = i8::from_be_bytes(self.read_register8(reg_b).to_be_bytes()) as i16;

            let divisor = a.overflowing_div(b);

            self.write_register16(RegisterName::AccumulatorH, u16::from_be_bytes(divisor.0.to_be_bytes()));

            (divisor.0 == 0, (divisor.0 & 0x0080_i16) != 0, divisor.1)
        };

        // Carry/Borrow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0001, flags.2);
        // Zero flag
        self.register16_write_bits(RegisterName::StatusH, 0x0002, flags.0);
        // Sign flag
        self.register16_write_bits(RegisterName::StatusH, 0x0004, flags.1);
        // Overflow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0008, flags.2);
    }

    /// Increment register in-place

    fn inst_inc(&mut self, mem: &mut MemoryManager) {
        // This instruction is 2 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg = match self.fetch_register_operand(mem, ByteModeOverride::DontOverride) {
            Some(x) => x,
            None => return,
        };

        if !self.byte_mode_is_set() {
            self.write_register16(reg, self.read_register16(reg) + 1);
        } else {
            self.write_register8(reg, self.read_register8(reg) + 1);
        }
    }

    /// Decrement register in-place

    fn inst_dec(&mut self, mem: &mut MemoryManager) {
        // This instruction is 2 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg = match self.fetch_register_operand(mem, ByteModeOverride::DontOverride) {
            Some(x) => x,
            None => return,
        };

        if !self.byte_mode_is_set() {
            self.write_register16(reg, self.read_register16(reg) - 1);
        } else {
            self.write_register8(reg, self.read_register8(reg) - 1);
        }
    }

    /// Left-shift register and store the result in the Accumulator

    fn inst_lsh(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3-4 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The next 1-2 byte(s) are an immediate value
        if !self.byte_mode_is_set() {
            self.fetch_operand16(mem);
        } else {
            self.fetch_operand8(mem);
        }

        let reg = match reg {
            Some(x) => x,
            None => return,
        };

        if !self.byte_mode_is_set() {
            let shift = self.read_register16(RegisterName::InstructionH);

            self.write_register16(RegisterName::AccumulatorH, self.read_register16(reg) << shift);
        } else {
            let shift = self.read_register8(RegisterName::InstructionH);

            self.write_register16(RegisterName::AccumulatorH, (self.read_register8(reg) as u16) << shift);
        }
    }

    /// Right-shift register and store the result in the Accumulator

    fn inst_rsh(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3-4 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The next 1-2 byte(s) are an immediate value
        if !self.byte_mode_is_set() {
            self.fetch_operand16(mem);
        } else {
            self.fetch_operand8(mem);
        }

        let reg = match reg {
            Some(x) => x,
            None => return,
        };

        if !self.byte_mode_is_set() {
            let shift = self.read_register16(RegisterName::InstructionH);

            self.write_register16(RegisterName::AccumulatorH, self.read_register16(reg) >> shift);
        } else {
            let shift = self.read_register8(RegisterName::InstructionH);

            self.write_register8(RegisterName::AccumulatorL, self.read_register8(reg) >> shift);
        }
    }

    /// Bitwise-and two registers and store the result in the Accumulator

    fn inst_and(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg_a = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The third byte is a register
        let reg_b = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let reg_a = match reg_a {
            Some(x) => x,
            None => return,
        };

        let reg_b = match reg_b {
            Some(x) => x,
            None => return,
        };

        let flags = if !self.byte_mode_is_set() {
            let a = self.read_register16(reg_a);
            let b = self.read_register16(reg_b);

            let result = a & b;

            self.write_register16(RegisterName::AccumulatorH, result);

            (result == 0, (result & 0x8000) != 0)
        } else {
            let a = self.read_register8(reg_a);
            let b = self.read_register8(reg_b);

            let result = a & b;

            self.write_register8(RegisterName::AccumulatorL, result);

            (result == 0, (result & 0x80) != 0)
        };

        // Carry/Borrow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0001, false);
        // Zero flag
        self.register16_write_bits(RegisterName::StatusH, 0x0002, flags.0);
        // Sign flag
        self.register16_write_bits(RegisterName::StatusH, 0x0004, flags.1);
        // Overflow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0008, false);
    }

    /// Bitwise-or two registers and store the result in the Accumulator

    fn inst_or(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg_a = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The third byte is a register
        let reg_b = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let reg_a = match reg_a {
            Some(x) => x,
            None => return,
        };

        let reg_b = match reg_b {
            Some(x) => x,
            None => return,
        };

        let flags = if !self.byte_mode_is_set() {
            let a = self.read_register16(reg_a);
            let b = self.read_register16(reg_b);

            let result = a | b;

            self.write_register16(RegisterName::AccumulatorH, result);

            (result == 0, (result & 0x8000) != 0)
        } else {
            let a = self.read_register8(reg_a);
            let b = self.read_register8(reg_b);

            let result = a | b;

            self.write_register8(RegisterName::AccumulatorL, result);

            (result == 0, (result & 0x80) != 0)
        };

        // Carry/Borrow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0001, false);
        // Zero flag
        self.register16_write_bits(RegisterName::StatusH, 0x0002, flags.0);
        // Sign flag
        self.register16_write_bits(RegisterName::StatusH, 0x0004, flags.1);
        // Overflow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0008, false);
    }

    /// Bitwise-xor two registers and store the result in the Accumulator

    fn inst_xor(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg_a = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The third byte is a register
        let reg_b = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let reg_a = match reg_a {
            Some(x) => x,
            None => return,
        };

        let reg_b = match reg_b {
            Some(x) => x,
            None => return,
        };

        let flags = if !self.byte_mode_is_set() {
            let a = self.read_register16(reg_a);
            let b = self.read_register16(reg_b);

            let result = a ^ b;

            self.write_register16(RegisterName::AccumulatorH, result);

            (result == 0, (result & 0x8000) != 0)
        } else {
            let a = self.read_register8(reg_a);
            let b = self.read_register8(reg_b);

            let result = a ^ b;

            self.write_register8(RegisterName::AccumulatorL, result);

            (result == 0, (result & 0x80) != 0)
        };

        // Carry/Borrow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0001, false);
        // Zero flag
        self.register16_write_bits(RegisterName::StatusH, 0x0002, flags.0);
        // Sign flag
        self.register16_write_bits(RegisterName::StatusH, 0x0004, flags.1);
        // Overflow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0008, false);
    }

    /// Bitwise-not a register and store the result in the Accumulator

    fn inst_not(&mut self, mem: &mut MemoryManager) {
        // This instruction is 2 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let reg = match reg {
            Some(x) => x,
            None => return,
        };

        let flags = if !self.byte_mode_is_set() {
            let result = !self.read_register16(reg);

            self.write_register16(RegisterName::AccumulatorH, result);

            (result == 0, (result & 0x8000) != 0)
        } else {
            let result = !self.read_register8(reg);

            self.write_register8(RegisterName::AccumulatorL, result);

            (result == 0, (result & 0x80) != 0)
        };

        // Carry/Borrow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0001, false);
        // Zero flag
        self.register16_write_bits(RegisterName::StatusH, 0x0002, flags.0);
        // Sign flag
        self.register16_write_bits(RegisterName::StatusH, 0x0004, flags.1);
        // Overflow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0008, false);
    }

    /// Set bits in a register in-place
    
    fn inst_stb(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second byte is an 8-bit-only register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::ForceOn);

        // The third byte is an 8-bit-only immediate value
        let bitmask = self.fetch_operand8(mem);

        let reg = match reg {
            Some(x) => x,
            None => return,
        };

        self.write_register8(reg, self.read_register8(reg) | bitmask);
    }

    /// Clear bits in a register in-place
    
    fn inst_clb(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second byte is an 8-bit-only register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::ForceOn);

        // The third byte is an 8-bit-only immediate value
        let bitmask = self.fetch_operand8(mem);

        let reg = match reg {
            Some(x) => x,
            None => return,
        };

        self.write_register8(reg, self.read_register8(reg) & !bitmask);
    }

    /// Test bits in a register and store the result to the Accumulator
    
    fn inst_tsb(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second byte is an 8-bit-only register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::ForceOn);

        // The third byte is an 8-bit-only immediate value
        let bitmask = self.fetch_operand8(mem);

        let reg = match reg {
            Some(x) => x,
            None => return,
        };

        let result = (self.read_register8(reg) & bitmask) as u16;

        self.write_register16(RegisterName::AccumulatorH, result);

        // Zero flag
        self.register16_write_bits(RegisterName::StatusH, 0x0002, result == 0);
    }

    /// Jump to an address
    /// 
    /// All jump instructions prefix thin addresses with IndexB

    fn inst_jmp(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second and third bytes are a thin memory address
        let address = self.fetch_operand16(mem);

        // When making a wide address from a thin address, the high 2 bytes for addresses related to code are represented with the Index B register, whereas addresses related to data are represented with the Index A register
        self.copy_register16(RegisterName::IndexBH, RegisterName::ProgramCounterWH);
        self.write_register16(RegisterName::ProgramCounterH, address);
    }

    /// Jump to a relative address

    fn inst_jmpr(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second and third bytes are a thin memory address
        let address = i16::from_be_bytes((self.fetch_operand16(mem)).to_be_bytes()) as i32;

        let pc = i32::from_be_bytes(self.read_register32(RegisterName::ProgramCounterWH).to_be_bytes());

        self.write_register32(RegisterName::ProgramCounterWH, (pc + address) as u32);
    }

    /// Jump to a pointer

    fn inst_jmpp(&mut self, mem: &mut MemoryManager) {
        // This instruction is 2 bytes long, the first byte is the opcode
        // The second byte is a pointer
        let pointer_reg = self.fetch_register_operand(mem, ByteModeOverride::ForceOff);

        let pointer_reg = match pointer_reg {
            Some(x) => x,
            None => return,
        };

        self.copy_register16(RegisterName::IndexBH, RegisterName::ProgramCounterWH);
        self.copy_register16(pointer_reg, RegisterName::ProgramCounterH);
    }

    /// Compare a register to an immediate and store the flags

    fn inst_cmpi(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3-4 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The next 1-2 byte(s) are an immediate value
        if !self.byte_mode_is_set() {
            self.fetch_operand16(mem);
        } else {
            self.fetch_operand8(mem);
        }

        let reg = match reg {
            Some(x) => x,
            None => return,
        };

        let flags = if !self.byte_mode_is_set() {
            let a = self.read_register16(reg);
            let b = self.read_register16(RegisterName::InstructionH);
            let signed_a = i16::from_be_bytes(a.to_be_bytes());
            let signed_b = i16::from_be_bytes(b.to_be_bytes());

            let difference = a.overflowing_sub(b);

            (difference.1, difference.0 == 0, (difference.0 & 0x8000) != 0, signed_a.overflowing_sub(signed_b).1)
        } else {
            let a = self.read_register8(reg);
            let b = self.read_register8(RegisterName::InstructionH);
            let signed_a = i8::from_be_bytes(a.to_be_bytes());
            let signed_b = i8::from_be_bytes(b.to_be_bytes());

            let difference = a.overflowing_sub(b);

            (difference.1, difference.0 == 0, (difference.0 & 0x80) != 0, signed_a.overflowing_sub(signed_b).1)
        };

        // Carry/Borrow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0001, flags.0);
        // Zero flag
        self.register16_write_bits(RegisterName::StatusH, 0x0002, flags.1);
        // Sign flag
        self.register16_write_bits(RegisterName::StatusH, 0x0004, flags.2);
        // Overflow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0008, flags.3);
    }

    /// Compare two registers and store the flags

    fn inst_cmpr(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second byte is a register
        let reg_a = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        // The third byte is a register
        let reg_b = self.fetch_register_operand(mem, ByteModeOverride::DontOverride);

        let reg_a = match reg_a {
            Some(x) => x,
            None => return,
        };

        let reg_b = match reg_b {
            Some(x) => x,
            None => return,
        };

        let flags = if !self.byte_mode_is_set() {
            let a = self.read_register16(reg_a);
            let b = self.read_register16(reg_b);
            let signed_a = i16::from_be_bytes(a.to_be_bytes());
            let signed_b = i16::from_be_bytes(b.to_be_bytes());

            let difference = a.overflowing_sub(b);

            (difference.1, difference.0 == 0, (difference.0 & 0x8000) != 0, signed_a.overflowing_sub(signed_b).1)
        } else {
            let a = self.read_register8(reg_a);
            let b = self.read_register8(reg_b);
            let signed_a = i8::from_be_bytes(a.to_be_bytes());
            let signed_b = i8::from_be_bytes(b.to_be_bytes());

            let difference = a.overflowing_sub(b);

            (difference.1, difference.0 == 0, (difference.0 & 0x80) != 0, signed_a.overflowing_sub(signed_b).1)
        };

        // Carry/Borrow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0001, flags.0);
        // Zero flag
        self.register16_write_bits(RegisterName::StatusH, 0x0002, flags.1);
        // Sign flag
        self.register16_write_bits(RegisterName::StatusH, 0x0004, flags.2);
        // Overflow flag
        self.register16_write_bits(RegisterName::StatusH, 0x0008, flags.3);
    }

    /// Jump to a relative address if the last comparison is equal, or if the zero flag is set

    fn inst_je(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second and third bytes are a thin memory address
        let address = i16::from_be_bytes((self.fetch_operand16(mem)).to_be_bytes()) as i32;

        let pc = i32::from_be_bytes(self.read_register32(RegisterName::ProgramCounterWH).to_be_bytes());

        let zero_flag = self.read_register16(RegisterName::StatusH) & 0x0002 != 0;

        if zero_flag {
            self.write_register32(RegisterName::ProgramCounterWH, (pc + address) as u32);
        }
    }

    /// Jump to a relative address if the last comparison is not equal, or if the zero flag is not set

    fn inst_jne(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second and third bytes are a thin memory address
        let address = i16::from_be_bytes((self.fetch_operand16(mem)).to_be_bytes()) as i32;

        let pc = i32::from_be_bytes(self.read_register32(RegisterName::ProgramCounterWH).to_be_bytes());

        let zero_flag = self.read_register16(RegisterName::StatusH) & 0x0002 != 0;

        if !zero_flag {
            self.write_register32(RegisterName::ProgramCounterWH, (pc + address) as u32);
        }
    }

    /// Jump to a relative address if the last comparison is greater

    fn inst_jg(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second and third bytes are a thin memory address
        let address = i16::from_be_bytes((self.fetch_operand16(mem)).to_be_bytes()) as i32;

        let pc = i32::from_be_bytes(self.read_register32(RegisterName::ProgramCounterWH).to_be_bytes());

        let zero_flag = self.read_register16(RegisterName::StatusH) & 0x0002 != 0;
        let sign_flag = self.read_register16(RegisterName::StatusH) & 0x0004 != 0;

        if !sign_flag && !zero_flag {
            self.write_register32(RegisterName::ProgramCounterWH, (pc + address) as u32);
        }
    }

    /// Jump to a relative address if the last comparison is lesser

    fn inst_jl(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second and third bytes are a thin memory address
        let address = i16::from_be_bytes((self.fetch_operand16(mem)).to_be_bytes()) as i32;

        let pc = i32::from_be_bytes(self.read_register32(RegisterName::ProgramCounterWH).to_be_bytes());

        let sign_flag = self.read_register16(RegisterName::StatusH) & 0x0004 != 0;

        // A number can't be negative and be zero, the sign flag says if a number is negative and non-zero on its own
        if sign_flag {
            self.write_register32(RegisterName::ProgramCounterWH, (pc + address) as u32);
        }
    }

    /// Jump to a relative address if the last comparison is greater or equal

    fn inst_jge(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second and third bytes are a thin memory address
        let address = i16::from_be_bytes((self.fetch_operand16(mem)).to_be_bytes()) as i32;

        let pc = i32::from_be_bytes(self.read_register32(RegisterName::ProgramCounterWH).to_be_bytes());

        let sign_flag = self.read_register16(RegisterName::StatusH) & 0x0004 != 0;

        // When a number is 0, the most significant bit will be 0, so if the sign flag is not set then that inherently means that the number is 0 or higher
        if !sign_flag {
            self.write_register32(RegisterName::ProgramCounterWH, (pc + address) as u32);
        }
    }

    /// Jump to a relative address if the last comparison is lesser or equal

    fn inst_jle(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second and third bytes are a thin memory address
        let address = i16::from_be_bytes((self.fetch_operand16(mem)).to_be_bytes()) as i32;

        let pc = i32::from_be_bytes(self.read_register32(RegisterName::ProgramCounterWH).to_be_bytes());

        let zero_flag = self.read_register16(RegisterName::StatusH) & 0x0002 != 0;
        let sign_flag = self.read_register16(RegisterName::StatusH) & 0x0004 != 0;

        if sign_flag || zero_flag {
            self.write_register32(RegisterName::ProgramCounterWH, (pc + address) as u32);
        }
    }

    /// Jump to a relative address if the last addition/subtraction set the carry/borrow flag

    fn inst_jc(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second and third bytes are a thin memory address
        let address = i16::from_be_bytes((self.fetch_operand16(mem)).to_be_bytes()) as i32;

        let pc = i32::from_be_bytes(self.read_register32(RegisterName::ProgramCounterWH).to_be_bytes());

        let carry_flag = self.read_register16(RegisterName::StatusH) & 0x0001 != 0;

        if carry_flag {
            self.write_register32(RegisterName::ProgramCounterWH, (pc + address) as u32);
        }
    }

    /// Jump to a relative address if the last addition/subtraction did not set the carry/borrow flag

    fn inst_jnc(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second and third bytes are a thin memory address
        let address = i16::from_be_bytes((self.fetch_operand16(mem)).to_be_bytes()) as i32;

        let pc = i32::from_be_bytes(self.read_register32(RegisterName::ProgramCounterWH).to_be_bytes());

        let carry_flag = self.read_register16(RegisterName::StatusH) & 0x0001 != 0;

        if !carry_flag {
            self.write_register32(RegisterName::ProgramCounterWH, (pc + address) as u32);
        }
    }

    /// Jump to a relative address if the last addition/subtraction set the overflow flag

    fn inst_jo(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second and third bytes are a thin memory address
        let address = i16::from_be_bytes((self.fetch_operand16(mem)).to_be_bytes()) as i32;

        let pc = i32::from_be_bytes(self.read_register32(RegisterName::ProgramCounterWH).to_be_bytes());

        let overflow_flag = self.read_register16(RegisterName::StatusH) & 0x0008 != 0;

        if overflow_flag {
            self.write_register32(RegisterName::ProgramCounterWH, (pc + address) as u32);
        }
    }

    /// Jump to a relative address if the last addition/subtraction did not set the overflor flag

    fn inst_jno(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second and third bytes are a thin memory address
        let address = i16::from_be_bytes((self.fetch_operand16(mem)).to_be_bytes()) as i32;

        let pc = i32::from_be_bytes(self.read_register32(RegisterName::ProgramCounterWH).to_be_bytes());

        let overflow_flag = self.read_register16(RegisterName::StatusH) & 0x0008 != 0;

        if !overflow_flag {
            self.write_register32(RegisterName::ProgramCounterWH, (pc + address) as u32);
        }
    }

    /// Call to a subroutine at an address
    /// 
    /// All subroutine instructions prefix thin addresses with IndexB
    
    fn inst_cla(&mut self, mem: &mut MemoryManager) {
        // This instruction is 3 bytes long, the first byte is the opcode
        // The second and third bytes are a thin memory address
        self.fetch_operand16(mem);
        
        self.push16(mem, self.read_register16(RegisterName::R3H));
        self.push16(mem, self.read_register16(RegisterName::R2H));
        self.push16(mem, self.read_register16(RegisterName::R1H));
        self.push16(mem, self.read_register16(RegisterName::R0H));
        self.push16(mem, self.read_register16(RegisterName::IndexBH));
        self.push16(mem, self.read_register16(RegisterName::IndexAH));
        self.push16(mem, self.read_register16(RegisterName::ProgramCounterH));
        self.push16(mem, self.read_register16(RegisterName::ProgramCounterWH));

        let frame_size = 2 + (self.read_register32(RegisterName::BasePointerWH) - self.read_register32(RegisterName::StackPointerWH)) as u16;

        self.push16(mem, frame_size);

        self.copy_register32(RegisterName::StackPointerWH, RegisterName::BasePointerWH);

        self.copy_register16(RegisterName::IndexBH, RegisterName::ProgramCounterWH);
        self.copy_register16(RegisterName::InstructionH, RegisterName::ProgramCounterH);
    }

    /// Call to a subroutine at a pointer
    
    fn inst_clp(&mut self, mem: &mut MemoryManager) {
        // This instruction is 2 bytes long, the first byte is the opcode
        // The second byte is a pointer
        let pointer_reg = self.fetch_register_operand(mem, ByteModeOverride::ForceOff);

        let pointer_reg = match pointer_reg {
            Some(x) => x,
            None => return,
        };

        self.push16(mem, self.read_register16(RegisterName::R3H));
        self.push16(mem, self.read_register16(RegisterName::R2H));
        self.push16(mem, self.read_register16(RegisterName::R1H));
        self.push16(mem, self.read_register16(RegisterName::R0H));
        self.push16(mem, self.read_register16(RegisterName::IndexBH));
        self.push16(mem, self.read_register16(RegisterName::IndexAH));
        self.push16(mem, self.read_register16(RegisterName::ProgramCounterH));
        self.push16(mem, self.read_register16(RegisterName::ProgramCounterWH));

        let frame_size = 2 + (self.read_register32(RegisterName::BasePointerWH) - self.read_register32(RegisterName::StackPointerWH)) as u16;

        self.push16(mem, frame_size);

        self.copy_register32(RegisterName::StackPointerWH, RegisterName::BasePointerWH);
        
        self.copy_register16(RegisterName::IndexBH, RegisterName::ProgramCounterWH);
        self.copy_register16(pointer_reg, RegisterName::ProgramCounterH);
    }

    /// Return from a subroutine
    
    fn inst_ret(&mut self, mem: &mut MemoryManager) {
        self.copy_register32(RegisterName::BasePointerWH, RegisterName::StackPointerWH);
        
        let frame_size = self.pop16(mem) as u32;

        self.write_register32(RegisterName::BasePointerWH, self.read_register32(RegisterName::BasePointerWH) + frame_size);

        let value = self.pop16(mem);
        self.write_register16(RegisterName::ProgramCounterWH, value);
        let value = self.pop16(mem);
        self.write_register16(RegisterName::ProgramCounterH, value);
        let value = self.pop16(mem);
        self.write_register16(RegisterName::IndexAH, value);
        let value = self.pop16(mem);
        self.write_register16(RegisterName::IndexBH, value);
        let value = self.pop16(mem);
        self.write_register16(RegisterName::R0H, value);
        let value = self.pop16(mem);
        self.write_register16(RegisterName::R1H, value);
        let value = self.pop16(mem);
        self.write_register16(RegisterName::R2H, value);
        let value = self.pop16(mem);
        self.write_register16(RegisterName::R3H, value);
    }

    /// Set byte mode

    fn inst_sbm(&mut self) {
        self.register16_write_bits(RegisterName::StatusH, 0x0020, true);
    }

    /// Clear byte mode

    fn inst_cbm(&mut self) {
        self.register16_write_bits(RegisterName::StatusH, 0x0020, false);
    }

    /// Halt
    
    fn inst_hlt(&mut self) {
        self.halt = true;
    }
}