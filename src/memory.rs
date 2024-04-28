use std::ops::Range;

/// A virtual "device" to be added to a [MemoryManager]
/// 
/// Is used to control a [CPU](super::cpu::CPU)'s access to a range of memory
/// 
/// If there are no [Device]s added to a [MemoryManager], or there are none that cover a certain range, that memory will be treated as ReadWrite

#[derive(Clone, PartialEq)]
pub struct Device {
    range: Range<usize>,
    pub access_type: AccessType,
}

impl Device {
    /// Constructs a new [Device]
    /// 
    /// Will panic if `range.start` >= `range.end`

    pub fn new(range: Range<usize>, access_type: AccessType) -> Self {
        if range.start >= range.end {
            panic!("range.start ({}) is >= range.end ({})", range.start, range.end);
        }

        Device {
            range,
            access_type,
        }
    }

    /// Returns the range that this [Device] covers
    
    pub fn range(&self) -> &Range<usize> {
        &self.range
    }
}


/// The access type a [Device] has

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum AccessType {
    ReadWrite,
    ReadOnly,
    WriteOnly,
}


/// A manager for the memory space usuable by the machine

#[derive(Clone)]
pub struct MemoryManager {
    data: Box<[u8]>,
    devices: Vec<Device>,
}

impl MemoryManager {
    /// Constructs a new `MemoryManager`
    /// 
    /// Will panic if `size == 0`
    
    pub fn new(size: usize) -> MemoryManager {
        if size == 0 {
            panic!("size == 0");
        }
        
        MemoryManager {
            data: vec![0x00_u8; size].into_boxed_slice(),
            devices: Vec::new(),
        }
    }

    /// Returns the size of memory

    pub fn size(&self) -> usize {
        self.data.len()
    }

    /// Resets all bytes in memory to zero
     
    pub fn reset(&mut self) {
        self.data.fill(0x00);
    }

    /// Adds a [Device]
    /// 
    /// Will panic if `device`'s range exceeds the size of the memory manager, or if `device`'s range overlaps a different [Device]'s range that doesn't have the same access type

    pub fn add_device(&mut self, device: Device) {
        if device.range.start >= self.data.len() {
            panic!("device's range exceeds the memory manager's size, range.start ({}) >= size ({})", device.range.start, self.data.len());
        }

        if device.range.end > self.data.len() {
            panic!("device's range exceeds the memory manager's size, range.end ({}) > size ({})", device.range.end, self.data.len());
        }

        for other_device in self.devices.iter() {
            if device.access_type != other_device.access_type && device.range.start < other_device.range.end && other_device.range.start < device.range.end {
                panic!("device's range ({:?}) overlaps with a different device's range ({:?}) and doesn't have the same access type ({:?}) as it has ({:?})", device.range, other_device.range, device.access_type, other_device.access_type);
            }
        }

        self.devices.push(device);
    }

    /// Returns [Some]\(&[Device]) if the memory manager has a device with a range that contains `index`
    /// 
    /// Returns [None] otherwise

    pub fn get_device_from_index(&self, index: usize) -> Option<&Device> {
        for device in self.devices.iter() {
            if device.range.contains(&index) {
                return Some(device)
            }
        }
        None
    }
    
    /// Returns the [u8] in memory at `index`
    /// 
    /// Returns 0 if `index` is >= memory size
    /// 
    /// Returns 0 if `index` is within the range of a WriteOnly device, unless `ignore_access_type` = true

    pub fn read_byte(&self, index: usize, ignore_access_type: bool) -> u8 {
        if !ignore_access_type {
            if let Some(device) = self.get_device_from_index(index) {
                if device.access_type == AccessType::WriteOnly {
                    return 0;
                }
            }
        }
        
        if index >= self.data.len() { 0 }
        else                        { self.data[index] }
    }

    /// Sets `index` of memory to `byte`
    /// 
    /// Won't do anything if `index` is >= memory size
    /// 
    /// Won't do anything if `index` is within the range of a ReadOnly device, unless `ignore_access_type` = true

    pub fn write_byte(&mut self, index: usize, byte: u8, ignore_access_type: bool) {
        if !ignore_access_type {
            if let Some(device) = self.get_device_from_index(index) {
                if device.access_type == AccessType::ReadOnly {
                    return;
                }
            }
        }

        if index < self.data.len() {
            self.data[index] = byte;
        }
    }

    /// Returns the \[[u8]; 2\] in memory at `index`
    /// 
    /// Any index >= memory size or within the range of a ReadOnly device will read 0

    pub fn read_byte2(&self, index: usize, ignore_access_type: bool) -> [u8; 2] {
        let mut bytes = [0x00; 2];

        bytes[0] = self.read_byte(index, ignore_access_type);
        bytes[1] = self.read_byte(index + 1, ignore_access_type);

        bytes
    }

    /// Sets `index..index + 2` of memory to `bytes`
    /// 
    /// Any byte written to an index >= memory size or to an index within the range of a ReadOnly device will be ignored

    pub fn write_byte2(&mut self, index: usize, bytes: [u8; 2], ignore_access_type: bool) {
        self.write_byte(index, bytes[0], ignore_access_type);
        self.write_byte(index + 1, bytes[1], ignore_access_type);
    }

    /// Returns the \[[u8]; 4\] in memory at `index`
    /// 
    /// Any index >= memory size or within the range of a ReadOnly device will read 0

    pub fn read_byte4(&self, index: usize, ignore_access_type: bool) -> [u8; 4] {
        let mut bytes = [0x00; 4];

        bytes[0] = self.read_byte(index, ignore_access_type);
        bytes[1] = self.read_byte(index + 1, ignore_access_type);
        bytes[2] = self.read_byte(index + 2, ignore_access_type);
        bytes[3] = self.read_byte(index + 3, ignore_access_type);

        bytes
    }

    /// Sets `index..index + 4` of memory to `bytes`
    /// 
    /// Any byte written to an index >= memory size or to an index within the range of a ReadOnly device will be ignored

    pub fn write_byte4(&mut self, index: usize, bytes: [u8; 4], ignore_access_type: bool) {
        self.write_byte(index, bytes[0], ignore_access_type);
        self.write_byte(index + 1, bytes[1], ignore_access_type);
        self.write_byte(index + 2, bytes[2], ignore_access_type);
        self.write_byte(index + 3, bytes[3], ignore_access_type);
    }

    /// Loads a \&[[u8]\] into memory at `index`
    /// 
    /// Will ignore the access type of the range of memory
    ///
    /// Any byte written to an index >= memory size will be ignored
     
    pub fn load_bytes(&mut self, index: usize, bytes: &[u8]) {
        for (i, byte) in bytes.iter().enumerate() {
            self.write_byte(index + i, *byte, true);
        }
    }

    /// Returns a [Box<\[u8\]>] of the bytes in `range` of memory
    /// 
    /// Will ignore the access type of the range of memory
    ///
    /// Any index >= memory size will read 0
     
    pub fn peek_bytes(&mut self, range: Range<usize>) -> Box<[u8]> {
        let mut bytes = Vec::with_capacity(range.len());
        
        for index in range {
            bytes.push(self.read_byte(index, true));
        }

        bytes.into_boxed_slice()
    }
}