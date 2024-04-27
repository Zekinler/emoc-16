mod memory;
mod cpu;

use cpu::{Cpu, RegisterName};
use memory::MemoryManager;
use std::io;

const ROM_SIZE: usize = 0x1000;     // 4 KiB of ROM
const WRAM_SIZE: usize = 0xF000;    // 60 KiB of Work RAM


/// Counts to 3 and stores the numbers at [4096]

const BRANCHING_TEST_PROGRAM: [u8; 23] = [
    0x03, 0x00, 0x10, 0x00,     // [0] Load Accumulator with [4096]
    0x01, 0x03, 0x00, 0x01,     // [4] Load R0 with 1
    0x10, 0x00, 0x03,           // [8] Add Accumulator with R0
    0x06, 0x10, 0x00, 0x00,     // [11] Store at 4096, Accumulator
    0x33, 0x00, 0x00, 0x03,     // [15] Compare Accumulator to 3
    0x36, 0xFF, 0xEA,           // [19] Relative jump -22 bytes if comparison is not equal
    0xFF                        // [22] Halt
];

/// Loads 0x8008 into a register, pushes it to the stack, adds 1 with the register, pops from the stack and subtracts that from the Accumulator, finally, stores the difference to [4096]

const STACK_TEST_PROGRAM: [u8; 23] = [
    0x0A, 0xFF, 0xFE,           // [0] Initialize Stack to 0xFFFE
    0x01, 0x03, 0x80, 0x08,     // [3] Load R0 with 0x8008
    0x0C, 0x03,                 // [7] Push R0
    0x11, 0x03, 0x00, 0x01,     // [9] Add R0 with 1
    0x0D, 0x03,                 // [13] Pop to R0
    0x12, 0x00, 0x03,           // [15] Subtract Accumulator with R0
    0x06, 0x10, 0x00, 0x00,     // [18] Store at 4096, Accumulator
    0xFF                        // [22] Halt
];

/// Messes with state and subroutines

const SUBROUTINE_TEST_PROGRAM: [u8; 27] = [
    0x0A, 0xFF, 0xFE,           // [0] Initalize Stack to 0xFFFE
    0x0B, 0x33, 0x33,           // [3] Push 0x3333 to stack
    0x0B, 0x22, 0x22,           // [6] Push 0x2222 to stack
    0x0B, 0x11, 0x11,           // [9] Push 0x1111 to stack
    0x01, 0x03, 0x12, 0x34,     // [12] Load 0x1234 into R0
    0x01, 0x06, 0x56, 0x78,     // [16] Load 0x5678 into R3
    0x40, 0x04, 0x00,           // [20] Call [1024]
    0x0B, 0x44, 0x44,           // [23] Push 0x4444 to stack
    0xFF                        // [26] Halt
];

const TEST_SUBROUTINE: [u8; 18] = [
    0x0B, 0x01, 0x02,           // [0] Push 0x0102 to stack
    0x0B, 0x03, 0x04,           // [3] Push 0x0304 to stack
    0x0B, 0x05, 0x06,           // [6] Push 0x0506 to stack
    0x01, 0x03, 0x07, 0x08,     // [9] Load 0x0708 into R0
    0x01, 0x06, 0x09, 0x0A,     // [13] Load 0x090A into R3
    0x42                        // [17] Return
];

#[allow(unused_variables)]
fn main() {
    let mut cpu = Cpu::new();

    let mut memory = MemoryManager::new(ROM_SIZE + WRAM_SIZE);

    let rom_pointer = memory.add_device(String::from("ROM"), memory::DeviceAccessMode::ReadOnly, 0..ROM_SIZE);

    let wram_pointer = memory.add_device(String::from("WRAM"), memory::DeviceAccessMode::ReadWrite, rom_pointer.end..(rom_pointer.end + WRAM_SIZE));

    memory.write_bytes(0, &SUBROUTINE_TEST_PROGRAM);
    memory.write_bytes(1024, &TEST_SUBROUTINE);

    let mut cycle = 0;

    while !cpu.is_halted() {
        let pc = cpu.read_register32(RegisterName::ProgramCounterWH);
        
        println!("{} [PC]{:04X},4 = 0x{:08X}", cycle, pc, memory.read32(pc as usize, true));

        let mut dummy = String::new();

        io::stdin()
            .read_line(&mut dummy)
            .expect("Failed to read line");

        
        cpu.clock(&mut memory);

        cycle += 1;

        println!("{} R0 = 0x{:04X}", cycle, cpu.read_register16(RegisterName::R0H));
        println!("{} R3 = 0x{:04X}", cycle, cpu.read_register16(RegisterName::R3H));
        println!("{} BP = 0x{:04X}", cycle, cpu.read_register32(RegisterName::BasePointerWH));

        let sp = (cpu.read_register32(RegisterName::StackPointerWH) + 2) as usize;

        println!("{} [SP + 2]{:04X},2 = 0x{:04X}", cycle, sp, memory.read16(sp, true));
        println!("");
    }

    println!("- Halted");

    println!("- [4096],2 = 0x{:04X}", memory.read16(0x1000, true));
}
