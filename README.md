# EMOC-16

## Introduction

A 16-bit Emulator for a Machine of my Own Creation.

I've always wanted to build my own computer, not as in put some off-the-shelf components in a case, but actually design a processor with my own instruction set.
Unfortunately, I do not have the money, equipment, or even know-how to do this on real silicon. However, what I do have is a personal computer, and the ability to code.

So off-and-on for years now, I've been making attempts at programming an emulator for a machine that doesn't yet exist, and this is my latest attempt.

## General Specs

The EMOC-16 is a 16-bit machine with 32-bit addressing space.

It has 18 16-bit registers.

### Registers

The EMOC-16 uses 16-bit registers. Some of them are grouped to make 32-bit "Wide" addresses, with 16-bit addresses being called "Thin" addresses.

Most cannot be targeted by instructions directly, some of these are only used internally by the CPU, while others have specific instructions for interacting with them.

| Codepoint   | Width | Name            | Purpose                                                                                                                           |
| ----------- | ----- | --------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| 0x00        | 16    | Accumulator     | Holds the results of math and logic operations (Unless the operation is in-place)                                                 |
| -           | 16    | Status          | Holds arithmetic, logic, and control flags                                                                                        |
| -           | 32    | Memory Address  | Holds the address that will be sent on the address bus to memory                                                                  |
| -           | 16    | Memory Data     | Holds the data that will be sent on the data bus, or that has been received from the data bus                                     |
| -           | 32    | Program Counter | Holds the address pointing to the current place in execution                                                                      |
| -           | 32    | Stack Pointer   | Holds the address pointing to the top of the stack                                                                                |
| -           | 32    | Base Pointer    | Holds the address pointing to the bottom of the current stack frame                                                               |
| 0x01        | 16    | IndexA          | Provides the high-half of a wide address for **data**-related instructions that take either a thin-address, or a register pointer |
| 0x02        | 16    | IndexB          | Provides the high-half of a wide address for **code**-related instructions that take either a thin-address, or a register pointer |
| 0x03        | 16    | R0              | General purpose register 0                                                                                                        |
| 0x04        | 16    | R1              | General purpose register 1                                                                                                        |
| 0x05        | 16    | R2              | General purpose register 2                                                                                                        |
| 0x06        | 16    | R3              | General purpose register 3                                                                                                        |

"Codepoint" is the number used to target a register in an instruction that takes a register operand, if a register does not have a codepoint, it cannot be targeted.

> The lower-halves of registers can only be targeted when Byte-Mode is enabled by setting the highest bit of the codepoint to 1.

"Width" is how many bits the register holds.

"Name" is a user-friendly name for the register.

"Purpose" describes what the register is used for.

The status flags are as such, H: `-.-.-.-.-.-.-.-` L: `-.-.B.I.O.S.Z.C`
- `C`: "Carry/Borrow",
- `Z`: "Zero",
- `S`: "Sign",
- `O`: "Overflow",
- `I`: "Interrupt Disable",
- `B`: "Byte Mode" (`0`: instructions use 16-bit values, or `1`: 8-bit values),
- `-`: unused