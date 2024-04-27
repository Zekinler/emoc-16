# EMOC-16

A 16-bit Emulator for a Machine of my Own Creation.
The name can also mean an Emulator for a Machine for *your* own Creations.

## Specs

This is a 16-bit machine with 32-bit addressing space.

### Registers

The EMOC-16 uses 16-bit registers. Some of them are grouped to make 32-bit "Wide" addresses, with 16-bit addresses being called "Thin" addresses.
Most cannot be targeted by instructions directly, some of these are only used internally by the CPU, while others have specific instructions for interacting with them.

Below is a table of the registers:

"Codepoint" is the number used to target a register in an instruction that takes a register operand, if a register does not have a codepoint, it cannot be targeted.
> The lower-halves of registers can only be targeted when Byte-Mode is enabled by setting the highest bit of the codepoint to 1.
"Width" is how many bits the register holds.
"Name" is a user-friendly name for the register.
"Purpose" describes what the register is used for.

| Codepoint   | Width | Name        | Purpose                                                                            |
| ----------- | ----- | ----------- | ---------------------------------------------------------------------------------- |
| 0x00        | 16    | Accumulator | Stores the results of math and logic operations (Unless the operation is in-place) |
| 0x01        | 16    | IndexA      | Provides the high-half of a wide address for certain data-related instructions that take either a thin-address, or a register pointer |