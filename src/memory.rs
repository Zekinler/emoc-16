use std::ops::Range;

/// Memory

pub struct Memory {
    pub read_only: bool,
    data: Box<[u8]>,
}

impl Memory {
    
    /// Constructs a new [Memory]

    pub fn new(read_only: bool, capacity: usize) -> Self {
        Self {
            read_only,
            data: vec![0x00_u8; capacity].into_boxed_slice(),
        }
    }

    /// Returns the size of memory

    pub fn size(&self) -> usize {
        self.data.len()
    }

    /// Resets all bytes in memory to zero
    
    pub fn reset(&mut self) {
        self.data.fill(0x00_u8);
    }

    /// Loads a &\[[u8]\] into memory at `index`
    /// 
    /// Panics if `bytes` wont fit into memory at `index`
     
    pub fn load(&mut self, index: usize, bytes: &[u8]) {
        if index + bytes.len() >= self.data.len() {
            panic!("bytes won't fit into memory at index: index + bytes.len() >= self.size()");
        }

        for (i, byte) in bytes.iter().enumerate() {
            self.data[index + i] = *byte;
        }
    }

    /// Returns a slice of the bytes in `range` of memory
     
    pub fn peek(&mut self, range: Range<usize>) -> &[u8] {
        &self.data[range]
    }

    /// Returns the [u8] in memory at `index`
    /// 
    /// Returns 0 if `index` >= memory size

    pub fn read_8(&self, index: usize) -> u8 {
        if index >= self.data.len() {
            0x00_u8
        } else {
            self.data[index]
        }
    }

    /// Returns the \[[u8]; 2\] in memory at `index`
    /// 
    /// Any byte outside of the memory size will be 0

    pub fn read_16(&self, index: usize) -> [u8; 2] {
        [
            self.read_8(index),
            self.read_8(index + 1)
        ]
    }

    /// Returns the \[[u8]; 4\] in memory at `index`
    /// 
    /// Any byte outside of the memory size will be 0

    pub fn read_32(&self, index: usize) -> [u8; 4] {
        [
            self.read_8(index),
            self.read_8(index + 1),
            self.read_8(index + 2),
            self.read_8(index + 3)
        ]
    }

    /// Writes `byte` into memory at `index`
    /// 
    /// Does nothing if memory is readonly or `index` >= memory size

    pub fn write_8(&mut self, index: usize, byte: u8) {
        if index >= self.data.len() || self.read_only {
            return
        }
        
        self.data[index] = byte;
    }

    /// Writes `bytes` into memory at `index`
    /// 
    /// Does nothing if memory is readonly
    /// 
    /// A byte won't be written if its index >= memory size

    pub fn write_16(&mut self, index: usize, bytes: [u8; 2]) {
        if self.read_only {
            return
        }

        self.write_8(index, bytes[0]);
        self.write_8(index + 1, bytes[1]);
    }

    /// Writes `bytes` into memory at `index`
    /// 
    /// Does nothing if memory type is ReadOnly
    /// 
    /// A byte won't be written if its index >= memory size

    pub fn write_32(&mut self, index: usize, bytes: [u8; 4]) {
        if self.read_only {
            return
        }

        self.write_8(index, bytes[0]);
        self.write_8(index + 1, bytes[1]);
        self.write_8(index + 2, bytes[2]);
        self.write_8(index + 3, bytes[3]);
    }
}