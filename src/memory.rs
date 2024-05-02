use std::ops::Range;


/// How memory can be accessed

#[derive(Clone, PartialEq)]
pub enum MemoryType {
    ReadWrite,
    ReadOnly,
}

/// Memory device

#[derive(Clone, PartialEq)]
pub struct Memory {
    memory_type: MemoryType,
    data: Box<[u8]>,
}

impl Memory {
    
    /// Constructs a new [Memory]

    pub fn new(memory_type: MemoryType, capacity: usize) -> Self {
        let mut vec = Vec::with_capacity(capacity);
        vec.fill(0x00_u8);

        Self {
            memory_type,
            data: vec.into_boxed_slice(),
        }
    }

    /// Returns the size of memory

    pub fn size(&self) -> usize {
        self.data.len()
    }

    /// Returns the [MemoryType] of memory
    
    pub fn memory_type(&self) -> MemoryType {
        self.memory_type
    }

    /// Resets all bytes in memory to zero
    /// 
    /// Does nothing if memory type isnt ReadWrite
     
    pub fn reset(&mut self) {
        if self.memory_type == MemoryType::ReadWrite {
            self.data.fill(0x00_u8);
        }
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
     
    pub fn peek_bytes(&mut self, range: Range<usize>) -> &[u8] {
        &self.data[range]
    }

    /// Returns the [u8] in memory at `index`
    /// 
    /// Returns 0 if `index` >= memory size

    pub fn read_byte(&self, index: usize) -> u8 {
        if index >= self.data.len() {
            0x00_u8
        } else {
            self.data[index]
        }
    }

    /// Returns the \[[u8]; 2\] in memory at `index`
    /// 
    /// Any byte outside of the memory size will be 0

    pub fn read_2bytes(&self, index: usize) -> [u8; 2] {
        [
            self.read_byte(index),
            self.read_byte(index + 1)
        ]
    }

    /// Returns the \[[u8]; 4\] in memory at `index`
    /// 
    /// Any byte outside of the memory size will be 0

    pub fn read_4bytes(&self, index: usize) -> [u8; 4] {
        [
            self.read_byte(index),
            self.read_byte(index + 1),
            self.read_byte(index + 2),
            self.read_byte(index + 3)
        ]
    }

    /// Writes `byte` into memory at `index`
    /// 
    /// Does nothing if memory type is ReadOnly or `index` >= memory size

    pub fn write_byte(&self, index: usize, byte: u8) {
        if self.memory_type == MemoryType::ReadOnly || index >= self.data.len() {
            return
        }
        
        self.data[index] = byte;
    }

    /// Writes `bytes` into memory at `index`
    /// 
    /// Does nothing if memory type is ReadOnly
    /// 
    /// A byte won't be written if its index >= memory size

    pub fn write_2bytes(&self, index: usize, bytes: [u8; 2]) {
        if self.memory_type == MemoryType::ReadOnly {
            return
        }

        self.write_byte(index, bytes[0]);
        self.write_byte(index + 1, bytes[1]);
    }

    /// Writes `bytes` into memory at `index`
    /// 
    /// Does nothing if memory type is ReadOnly
    /// 
    /// A byte won't be written if its index >= memory size

    pub fn write_4bytes(&self, index: usize, bytes: [u8; 4]) {
        if self.memory_type == MemoryType::ReadOnly {
            return
        }

        self.write_byte(index, bytes[0]);
        self.write_byte(index + 1, bytes[1]);
        self.write_byte(index + 2, bytes[2]);
        self.write_byte(index + 3, bytes[3]);
    }
}


/// A mapper for addressing multiple memory devices

#[derive(Clone)]
pub struct MemoryMapper {
    devices: Vec<(Memory, Range<usize>)>,
}

impl MemoryMapper {

    /// Constructs a new [MemoryMapper]
    
    pub fn new() -> MemoryMapper {
        MemoryMapper {
            devices: Vec::new()
        }
    }

    /// Adds a [Memory] device to the mapper

    pub fn add_device(&mut self, device: Memory) {
        let range = if self.devices.len() == 0 {
            0..device.size()
        } else {
            let start = self.devices.last().expect("devices should have a device if length != 0").1.end;
            
            start..start + device.size()
        };
        
        self.devices.push((device, range));
    }

    /// Returns [Some]\((&[Memory], usize)) if the memory mapper has a memory device with a range that contains `index`
    /// 
    /// The first member of the return tuple is a reference to the memory, the second is the mapped index
    /// 
    /// Returns [None] otherwise

    pub fn get_device_at(&self, index: usize) -> Option<(&Memory, usize)> {
        for device in self.devices.iter() {
            if device.1.contains(&index) {
                return Some((&device.0, index - device.1.start))
            }
        }
        None
    }

    /// Returns [Some]\((&mut [Memory], usize)) if the memory mapper has a memory device with a range that contains `index`
    /// 
    /// The first member of the return tuple is a mutable reference to the memory, the second is the mapped index
    /// 
    /// Returns [None] otherwise

    pub fn get_mut_device_at(&self, index: usize) -> Option<(&mut Memory, usize)> {
        for device in self.devices.iter() {
            if device.1.contains(&index) {
                return Some((&mut device.0, index - device.1.start))
            }
        }
        None
    }
}