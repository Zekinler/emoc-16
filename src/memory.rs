pub struct Memory {
  data: Box<[u8]>,
  size: usize,
}

impl Memory {
  pub fn new(size: usize) -> Memory {
    Memory {
      data: vec![b'\0'; size].into_boxed_slice(),
      size: size,
    }
  }

  pub fn size(&self) -> usize {
    self.size
  }

  pub fn load(&mut self, index: usize, data: &[u8]) {
    for (i, byte) in data.iter().enumerate() {
      self.data[index + i] = *byte;
    }
  }

  pub fn read_bytes(&self, index: usize, amount: usize) -> Vec<u8> {
    let mut data = vec![b'\0'; amount];

    let mut i = 0;
    while i < amount {
        data[i] = self.read8(index + i);
        i += 1;
    }

    data
  }


  pub fn read8(&self, index: usize) -> u8 {
    if index < self.size {
      return self.data[index]
    } else {
      return 0
    }
  }

  pub fn read16(&self, index: usize) -> u16 {
    let a = if index < self.size { (self.data[index] as u16) << 8 } else { 0 };
    let b = if index + 1 < self.size { self.data[index + 1] as u16 } else { 0 };

    a | b
  }
  
  pub fn read32(&self, index: usize) -> u32 {
    let a = if index < self.size { (self.data[index] as u32) << 24 } else { 0 };
    let b = if index + 1 < self.size { (self.data[index + 1] as u32) << 16 } else { 0 };
    let c = if index + 2 < self.size { (self.data[index + 2] as u32) << 8 } else { 0 };
    let d = if index + 3 < self.size { self.data[index + 3] as u32 } else { 0 };

    a | b | c | d
  }
  

  pub fn write8(&mut self, index: usize, value: u8) {
    if index < self.size {
      self.data[index] = value;
    }
  }

  pub fn write16(&mut self, index: usize, value: u16) {
    if index < self.size {
      self.data[index] = (value >> 8) as u8;
    }
    if index + 1 < self.size {
      self.data[index + 1] = value as u8;
    }
  }
  
  pub fn write32(&mut self, index: usize, value: u32) {
    if index < self.size {
      self.data[index] = (value >> 24) as u8;
    }
    if index + 1 < self.size {
      self.data[index + 1] = (value >> 16) as u8;
    }
    if index + 2 < self.size {
      self.data[index + 2] = (value >> 8) as u8;
    }
    if index + 3 < self.size {
      self.data[index + 3] = value as u8;
    }
  }
}