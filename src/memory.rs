#![allow(dead_code)]

pub enum DeviceAccessMode {
    ReadWrite,
    ReadOnly,
    WriteOnly,
}

struct Device {
    name: String,
    access_mode: DeviceAccessMode,
    range: std::ops::Range<usize>,
}

pub struct MemoryManager {
    data: Box<[u8]>,
    devices: Vec<Device>,
}

impl MemoryManager {
    pub fn new(size: usize) -> MemoryManager {
        if size < 1 {
            panic!("Can't initialize with a size of 0");
        }
        
        MemoryManager {
            data: vec![0x00; size].into_boxed_slice(),
            devices: Vec::new(),
        }
    }

    pub fn add_device(&mut self, name: String, access_mode: DeviceAccessMode, range: std::ops::Range<usize>) -> std::ops::Range<usize> {
        if range.start > self.data.len() {
            panic!("Start of range: {}, is larger than size of memory: {}", range.start, self.data.len());
        }

        if range.end > self.data.len() {
            panic!("End of range: {}, is larger than size of memory: {}", range.end, self.data.len());
        }
        
        if range.start > range.end {
            panic!("Start of range: {}, is larger than end of range: {}", range.start, range.end);
        }

        if range.end == range.start {
            panic!("End of range: {}, is equal to start of range: {}", range.end, range.start);
        }

        for device in self.devices.iter() {
            if name == device.name {
                panic!("Device with name: {} already exists", name);
            }

            if range.start >= device.range.end || (range.start < device.range.start && range.end <= device.range.start) {
                continue;
            }

            panic!("Range: {}..{}, is already covered by a different device: \"{}\" with range: {}..{}", range.start, range.end, device.name, device.range.start, device.range.end);
        }
        
        let ret_range = range.clone();

        let device = Device {
            name,
            access_mode,
            range,
        };

        self.devices.push(device);

        ret_range
    }

    pub fn size(&self) -> usize {
        self.data.len()
    }

    fn get_device_from_index(&self, index: usize) -> Option<&Device> {
        for device in self.devices.iter() {
            if device.range.contains(&index) {
                return Some(device);
            }
        }
        None
    }

    pub fn write_bytes(&mut self, index: usize, data: &[u8]) {
        for (i, byte) in data.iter().enumerate() {
            self.data[index + i] = *byte;
        }
    }

    pub fn read_bytes(&self, index: usize, amount: usize) -> &[u8] {
        &self.data[index..index + amount]
    }


    pub fn read8(&self, index: usize, force: bool) -> u8 {
        if !force {
            if let Some(device) = self.get_device_from_index(index) {
                match device.access_mode {
                    DeviceAccessMode::WriteOnly => return 0,
                    _ => (),
                }
            }
        }
        
        if index >= self.data.len() { 0 }
        else                        { self.data[index] }
    }

    pub fn read16(&self, index: usize, force: bool) -> u16 {
        let a = self.read8(index, force) as u16;
        let b = self.read8(index + 1, force) as u16;

        a << 8 | b
    }
  
    pub fn read32(&self, index: usize, force: bool) -> u32 {
        let a = self.read16(index, force) as u32;
        let b = self.read16(index + 2, force) as u32;

        a << 16 | b
    }

    pub fn read64(&self, index: usize, force: bool) -> u64 {
        let a = self.read32(index, force) as u64;
        let b = self.read32(index + 4, force) as u64;

        a << 32 | b
    }

    pub fn write8(&mut self, index: usize, value: u8, force: bool) {
        if !force {
            if let Some(device) = self.get_device_from_index(index) {
                match device.access_mode {
                    DeviceAccessMode::ReadOnly => return,
                    _ => (),
                }
            }
        }

        if index < self.size() {
            self.data[index] = value;
        }
    }

    pub fn write16(&mut self, index: usize, value: u16, force: bool) {
        let a = (value >> 8) as u8;
        let b = value as u8;

        self.write8(index, a, force);
        self.write8(index + 1, b, force);
    }
  
    pub fn write32(&mut self, index: usize, value: u32, force: bool) {
        let a = (value >> 16) as u16;
        let b = value as u16;

        self.write16(index, a, force);
        self.write16(index + 2, b, force);
    }

    pub fn write64(&mut self, index: usize, value: u64, force: bool) {
        let a = (value >> 32) as u32;
        let b = value as u32;

        self.write32(index, a, force);
        self.write32(index + 4, b, force);
    }
}