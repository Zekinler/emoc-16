mod memory;
mod cpu;

use cpu::CPU;

fn main() {
	let mut cpu = CPU::new(0x20000000);	// 512 MiB ram

	// This counts to 3 and stores it into the address location 256
	cpu.ram.load(0, &[
		0x16, 0x00, 0x00, 0x00, 0x01,	// Move into GA, 1
		0x17, 0x00, 0x00, 0x00, 0x03,	// Move into GB, 3
		0x21, 0x00, 0x00, 0x01, 0x00,	// Load to ACC from Adrloc. 256
		0x00, 0x16,						// Add ACC and GA
		0x31, 0x00, 0x00, 0x01, 0x00,	// Store from ACC to Adrloc. 256
		0x09, 0x17,						// Unsigned compare ACC and GB
		0x54, 0xED,						// Jump a byte if not equal, -19 bytes (0xED is -19 as a signed byte)
		0xFF,							// Halt
	]);

	cpu.run();

	println!("256:4 = {}", cpu.ram.read32(256));	// Printout Adrloc. 256 after it's done
}