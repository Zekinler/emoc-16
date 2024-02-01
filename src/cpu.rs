use crate::memory::Memory;

pub struct CPU {
	pub ram: Memory,
	registers: Memory,
	halt: bool,
	fl_equal: bool,
	fl_greater: bool,
	fl_less: bool,
}

impl CPU {
	pub fn new(ram_size: usize) -> CPU {
		CPU {
			ram: Memory::new(ram_size),
			registers: Memory::new(8 * 4),	// 8 registers with 4 bytes each
											// 00-03: Program Counter
											// 04-07: Accumulator
											// 08-11: Stack Pointer
											// 12-15: Base Pointer
											// 16-19: Input/Output Data
											// 20-23: Input/Output Address
											// 24-27: General A
											// 28-31: General B
			halt: false,
			fl_equal: false,
			fl_greater: false,
			fl_less: false,
		}
	}

    pub fn run(&mut self) {
        while !self.halt {
            let pc = self.registers.read32(0);
            println!("PC: {} = {:#04X}", pc, self.ram.read8(pc as usize));	// Printout Adrloc. program counter
    
            self.tick();
        }
    }

	pub fn tick(&mut self) {
		let opcode = self.fetch8();
		self.execute(opcode);
	}

    pub fn read_register(&self, register: usize) -> u32 {
        self.registers.read32(register * 4)
    }


	fn fetch8(&mut self) -> u8 {
		let pc = self.registers.read32(0);
		self.registers.write32(0, pc + 1);

		self.ram.read8(pc as usize)
	}

	fn fetch16(&mut self) -> u16 {
		let pc = self.registers.read32(0);
		self.registers.write32(0, pc + 2);
		
		self.ram.read16(pc as usize)
	}

	fn fetch32(&mut self) -> u32 {
		let pc = self.registers.read32(0);
		self.registers.write32(0, pc + 4);
		
		self.ram.read32(pc as usize)
	}


	fn execute(&mut self, opcode: u8) {
		match opcode {
			0x00 => {	// Add two registers. Stores the sum in ACC
				let regs = self.fetch8();
				
				let a = ((regs & 0xF0) >> 4) * 4;
				let b = (regs & 0x0F) * 4;

				let result = self.registers.read32(a as usize) + self.registers.read32(b as usize);
				self.registers.write32(4, result as u32);
			},
			0x01 => {	// Subtract two registers. Stores the difference in ACC
				let regs = self.fetch8();

				let a = ((regs & 0xF0) >> 4) * 4;
				let b = (regs & 0x0F) * 4;

				let result = self.registers.read32(a as usize) as i32 - self.registers.read32(b as usize) as i32;
				self.registers.write32(4, result as u32);
			},
			0x02 => {	// Multiply two registers. Stores the product in ACC
				let regs = self.fetch8();

				let a = ((regs & 0xF0) >> 4) * 4;
				let b = (regs & 0x0F) * 4;

				let result = self.registers.read32(a as usize) as i32 * self.registers.read32(b as usize) as i32;
				self.registers.write32(4, result as u32);
			},
			0x03 => {	// Integer divide two registers. Stores the product in ACC
				let regs = self.fetch8();

				let a = ((regs & 0xF0) >> 4) * 4;
				let b = (regs & 0x0F) * 4;

				let result = self.registers.read32(a as usize) as i32 / self.registers.read32(b as usize) as i32;
				self.registers.write32(4, result as u32);
			},
			0x04 => {	// Remainder two registers. Stores the product in ACC
				let regs = self.fetch8();

				let a = ((regs & 0xF0) >> 4) * 4;
				let b = (regs & 0x0F) * 4;

				let result = self.registers.read32(a as usize) as i32 % self.registers.read32(b as usize) as i32;
				self.registers.write32(4, result as u32);
			},
			0x05 => {	// Float add two registers. Stores the product in ACC
				let regs = self.fetch8();

				let a = ((regs & 0xF0) >> 4) * 4;
				let b = (regs & 0x0F) * 4;

				let result = f32::from_bits(self.registers.read32(a as usize)) + f32::from_bits(self.registers.read32(b as usize));
				self.registers.write32(4, result as u32);
			},
			0x06 => {	// Float subtract two registers. Stores the product in ACC
				let regs = self.fetch8();

				let a = ((regs & 0xF0) >> 4) * 4;
				let b = (regs & 0x0F) * 4;

				let result = f32::from_bits(self.registers.read32(a as usize)) - f32::from_bits(self.registers.read32(b as usize));
				self.registers.write32(4, result as u32);
			},
			0x07 => {	// Float multiply two registers. Stores the product in ACC
				let regs = self.fetch8();

				let a = ((regs & 0xF0) >> 4) * 4;
				let b = (regs & 0x0F) * 4;

				let result = f32::from_bits(self.registers.read32(a as usize)) * f32::from_bits(self.registers.read32(b as usize));
				self.registers.write32(4, result as u32);
			},
			0x08 => {	// Float divide two registers. Stores the product in ACC
				let regs = self.fetch8();

				let a = ((regs & 0xF0) >> 4) * 4;
				let b = (regs & 0x0F) * 4;

				let result = f32::from_bits(self.registers.read32(a as usize)) / f32::from_bits(self.registers.read32(b as usize));
				self.registers.write32(4, result as u32);
			},
			0x09 => {	// Unsigned compare two registers
				let regs = self.fetch8();

				let a = ((regs & 0xF0) >> 4) * 4;
				let b = (regs & 0x0F) * 4;

				self.fl_equal = self.registers.read32(a as usize) == self.registers.read32(b as usize);
				self.fl_greater = self.registers.read32(a as usize) > self.registers.read32(b as usize);
				self.fl_less = self.registers.read32(a as usize) < self.registers.read32(b as usize);
			},
			0x0A => {	// Signed compare two registers
				let regs = self.fetch8();

				let a = ((regs & 0xF0) >> 4) * 4;
				let b = (regs & 0x0F) * 4;

				self.fl_equal = self.registers.read32(a as usize) as i32 == self.registers.read32(b as usize) as i32;
				self.fl_greater = self.registers.read32(a as usize) as i32 > self.registers.read32(b as usize) as i32;
				self.fl_less = (self.registers.read32(a as usize) as i32) < self.registers.read32(b as usize) as i32;
			},
			0x0B => {	// Float compare two registers
				let regs = self.fetch8();

				let a = ((regs & 0xF0) >> 4) * 4;
				let b = (regs & 0x0F) * 4;

				self.fl_equal = f32::from_bits(self.registers.read32(a as usize)) == f32::from_bits(self.registers.read32(b as usize));
				self.fl_greater = f32::from_bits(self.registers.read32(a as usize)) > f32::from_bits(self.registers.read32(b as usize));
				self.fl_less = f32::from_bits(self.registers.read32(a as usize)) < f32::from_bits(self.registers.read32(b as usize));
			},

			0x10 => {	// Move into PC, an imm32 (or jump)
				let immediate = self.fetch32();

				self.registers.write32(0, immediate);
			},
			0x11 => {	// Move into ACC, an imm32
				let immediate = self.fetch32();

				self.registers.write32(4, immediate);
			},
			0x12 => {	// Move into SP, an imm32
				let immediate = self.fetch32();

				self.registers.write32(8, immediate);
			},
			0x13 => {	// Move into BP, an imm32
				let immediate = self.fetch32();

				self.registers.write32(12, immediate);
			},
			0x14 => {	// Move into IOD, an imm32
				let immediate = self.fetch32();

				self.registers.write32(16, immediate);
			},
			0x15 => {	// Move into IOA, an imm32
				let immediate = self.fetch32();

				self.registers.write32(20, immediate);
			},
			0x16 => {	// Move into GA, an imm32
				let immediate = self.fetch32();

				self.registers.write32(24, immediate);
			},
			0x17 => {	// Move into GB, an imm32
				let immediate = self.fetch32();

				self.registers.write32(28, immediate);
			},

			0x20 => {	// Load to PC, a memory address
				let address = self.fetch32();

				self.registers.write32(0, self.ram.read32(address as usize));
			},
			0x21 => {	// Load to ACC, a memory address
				let address = self.fetch32();

				self.registers.write32(4, self.ram.read32(address as usize));
			},
			0x22 => {	// Load to SP, a memory address
				let address = self.fetch32();

				self.registers.write32(8, self.ram.read32(address as usize));
			},
			0x23 => {	// Load to BP, a memory address
				let address = self.fetch32();

				self.registers.write32(12, self.ram.read32(address as usize));
			},
			0x24 => {	// Load to IOD, a memory address
				let address = self.fetch32();

				self.registers.write32(16, self.ram.read32(address as usize));
			},
			0x25 => {	// Load to IOA, a memory address
				let address = self.fetch32();

				self.registers.write32(20, self.ram.read32(address as usize));
			},
			0x26 => {	// Load to GA, a memory address
				let address = self.fetch32();

				self.registers.write32(24, self.ram.read32(address as usize));
			},
			0x27 => {	// Load to GB, a memory address
				let address = self.fetch32();

				self.registers.write32(28, self.ram.read32(address as usize));
			},

			0x30 => {	// Store from PC, a memory address
				let address = self.fetch32();

				self.ram.write32(address as usize, self.registers.read32(0));
			},
			0x31 => {	// Store from AC, a memory address
				let address = self.fetch32();

				self.ram.write32(address as usize, self.registers.read32(4));
			},
			0x32 => {	// Store from SP, a memory address
				let address = self.fetch32();

				self.ram.write32(address as usize, self.registers.read32(8));
			},
			0x33 => {	// Store from BP, a memory address
				let address = self.fetch32();

				self.ram.write32(address as usize, self.registers.read32(12));
			},
			0x34 => {	// Store from IOD, a memory address
				let address = self.fetch32();

				self.ram.write32(address as usize, self.registers.read32(16));
			},
			0x35 => {	// Store from IOA, a memory address
				let address = self.fetch32();

				self.ram.write32(address as usize, self.registers.read32(20));
			},
			0x36 => {	// Store from GA, a memory address
				let address = self.fetch32();

				self.ram.write32(address as usize, self.registers.read32(24));
			},
			0x37 => {	// Store from GB, a memory address
				let address = self.fetch32();

				self.ram.write32(address as usize, self.registers.read32(28));
			},

			0x40 => {	// Move into a register, a register
				let regs = self.fetch8();

				let a = ((regs & 0xF0) >> 4) * 4;
				let b = (regs & 0x0F) * 4;

				self.registers.write32(a as usize, self.registers.read32(b as usize));
			},
			0x41 => {	// Store into memory, an imm32
				let address = self.fetch32();

				let immediate = self.fetch32();

				self.ram.write32(address as usize, immediate);
			},
            0x42 => {   // Input from port
                
            },
            0x43 => {   // Output to port

            },

			0x50 => {	// Jump a byte
				let amount = self.fetch8() as i8;

				let pc = self.registers.read32(0);

				self.registers.write32(0, (pc as i32 - 2 + amount as i32) as u32);
			},
			0x51 => {	// Jump a byte if equal
				let amount = self.fetch8() as i8;

				if self.fl_equal {
					let pc = self.registers.read32(0);

					self.registers.write32(0, (pc as i32 - 2 + amount as i32) as u32);
				}
			},
			0x52 => {	// Jump a byte if greater
				let amount = self.fetch8() as i8;

				if self.fl_greater {
					let pc = self.registers.read32(0);

					self.registers.write32(0, (pc as i32 - 2 + amount as i32) as u32);
				}
			},
			0x53 => {	// Jump a byte if less
				let amount = self.fetch8() as i8;

				if self.fl_less {
					let pc = self.registers.read32(0);

					self.registers.write32(0, (pc as i32 - 2 + amount as i32) as u32);
				}
			},
			0x54 => {	// Jump a byte if not equal
				let amount = self.fetch8() as i8;

				if !self.fl_equal {
					let pc = self.registers.read32(0);

					self.registers.write32(0, (pc as i32 - 2 + amount as i32) as u32);
				}
			},
			0x55 => {	// Jump a byte if equal or greater
				let amount = self.fetch8() as i8;

				if !self.fl_less {
					let pc = self.registers.read32(0);

					self.registers.write32(0, (pc as i32 - 2 + amount as i32) as u32);
				}
			},
			0x56 => {	// Jump a byte if equal or less
				let amount = self.fetch8() as i8;

				if !self.fl_greater {
					let pc = self.registers.read32(0);

					self.registers.write32(0, (pc as i32 - 2 + amount as i32) as u32);
				}
			},

			0xFF => {	// Halt
				self.halt = true;
			},
			
			_ => (),    // NOOP
		}
	}
}