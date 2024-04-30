# EMOC-16

A 16-bit Emulator for a Machine of my Own Creation.

## Introduction

I've always wanted to build my own computer, not as in put some off-the-shelf components in a case, but actually design a processor with my own instruction set.  
Unfortunately, I do not have the money, equipment, or knowledge to do this on real silicon. However, what I do have is a computer, the ability to write code, and a spotty knowledge in CPU design.

So, I'm gonna program an emulator for this machine that doesn't exist! (and, yes, most of it is just winging-it as I learn more about how computers work)

## General Specs

The EMOC-16 has a 16-bit data bus, 32-bit address bus, and 512kB of RAM.

## Memory

Terminology:

- "Page" : The most-significant 16-bits of a "Full Address".
- "Full Address" : A 32-bit address that can be used to fully access the memory space.
- "Short Address" : A 16-bit register that is either relative to the current instruction (i.e. in a `JMP` instruction), or is combined with the Page register to make a full address.

Addressing Modes:

- "Immediate" : The address to read from memory is the address of the operand.
- "Direct" : The address to read from memory is in the operand.
- "Indirect" : The address to read from memory is at the address that is in the operand.
- "Indexed(X/Y)" : The address to read from memory is in the Index(X/Y) register + an offset that is in the operand.
- "Indexed(X/Y)-Indirect" : The address to read from memory is at (the address that is in the Index(X/Y) register + an offset that is in the operand).

All data is stored into memory in little-endian:

For example, take the instruction: `LDR` (LoaD Register) with "Direct" addressing mode, this instruction's first operand is the register to load into, but the second is a short address, let's say `0xE200`.  
This address would not be in the bytes after the instruction as `0xE2, 0x00`, instead it would appear as `0x00, 0xE2`, with the most-significant byte coming after the least-significant.  
The order of these bytes are swapped when they are loaded to the Memory Address register to read from memory, and, likewise, the data that is read from that address is in little-endian too, and the order is reversed before being loaded into the register.

### Mapping

The address space that the CPU can access is mapped to multiple devices, one device is the system's RAM and another is the ROM either of a cartridge or ROM built into the system.  
The ROM actually comes in two parts however, one part is always 256 bytes and mapped to the addresses `0x00..0xFF`. This part is called the "ROM Header" because it contains information needed to execute the ROM.

#### ROM Header

The first four bytes (addresses: `0x00..0x03`) are the address that Interrupt Requests will call to, as are the four bytes after that (`0x04..0x07`) the address that Non-maskable Interrupts will call to.  
The next four bytes after that (`0x08..0x0B`) are the address that the Program Counter will start at.

The rest of the space in the ROM Header can be used as the developer sees fit, either for subroutine calls, or for storing data.

#### RAM

Addresses: `0x0000_0100..0x0080_00FF` are mapped to the system's RAM of 512kB.

#### The Rest of the ROM

Addresses: `0x0080_0100..0x0080_0100 + ROM Size - 1` are mapped to the rest of the ROM's memory.

## Registers

The EMOC-16 has 5 general-use 16-bit registers, 5 internal 16-bit registers, 2 internal 32-bit address registers, and 1 internal 8-bit register.

The general-use registers can be used as operands for instructions that work on registers.
The internal registers are only used by the CPU, they are usually modified or tested by specific instructions, like `JMP` for the Program Counter, `CMP` or `JE` for the Status register, or `SP` for the Page register.

Instructions that take registers as arguments will take a "codepoint" to denote which register to select from:

| Codepoint   | Width | Name            | Purpose                                                                                                                           |
| ----------- | ----- | --------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| `0x00`      | 16    | Accumulator     | Holds the results of math and logic operations                                                                                    |
| Internal    | 8     | Status          | Holds arithmetic, logic, and control flags                                                                                        |
| Internal    | 32    | Memory Address  | Holds the address that will be sent on the address bus to memory                                                                  |
| Internal    | 16    | Memory Data     | Holds the data that will be sent on the data bus, or that has been received from the data bus                                     |
| Internal    | 32    | Program Counter | Holds the address pointing to the current place in execution                                                                      |
| Internal    | 16    | Instruction     | Holds the current instruction being executed                                                                                      |
| Internal    | 16    | Stack Pointer   | Holds the address pointing to next free space on the stack (The high 2 bytes are always 0x0001)                                   |
| Internal    | 16    | Frame Pointer   | Holds the address pointing to the bottom of the current stack frame (The high 2 bytes are always 0x0001)                          |
| Internal    | 16    | Page            | Holds the high 2 bytes of a full address (A "page") for absolute and indexed memory instructions                                  |
| `0x01`      | 16    | Index X         | Holds an indexing address for relative memory instructions                                                                        |
| `0x02`      | 16    | Index Y         | Holds an indexing address for relative memory instructions                                                                        |
| `0x03`      | 16    | R0              | General purpose register 0                                                                                                        |
| `0x04`      | 16    | R1              | General purpose register 1                                                                                                        |

### Status Flags

The Status register holds certain bitflags either denoting information about the previous instruction or controlling how the CPU works.

The Status flags are as such: `-.-.B.I.O.S.Z.C`

- `C`: "Carry/Borrow", whether the result of the last add or subtract instruction produced a carry bit or had to borrow past the most-significant bit
- `Z`: "Zero", whether the result of the last instruction was zero
- `S`: "Sign", whether the most-significant bit of the result is `1` (making it a negative number in two's-complement)
- `O`: "Overflow", whether the result of the last arithmetic instruction overflowed out of its bitwidth
- `I`: "Interrupt Disable", a control flag; whether the cpu will ignore Interrupt Requests
- `B`: "Byte Mode", a control flag; whether the cpu will process data on 8-bit values (a value of `1`) or 16-bit values (a value of `0`)
- `-`: Unused

When Byte Mode is enabled, the low-halves of general-use registers can be pointed to by setting the most-significant bit of the codepoint to `1` (i.e. when Byte Mode is `1`, codepoint: `0b0000_0011` = R0H, and `0b1000_0011` = R0L)
