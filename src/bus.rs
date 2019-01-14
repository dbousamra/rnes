use crate::cartridge::Cartridge;
use crate::mapper::Mapper;
use crate::util;

pub struct Bus {
  ram: [u8; 0x800],
  mapper: Box<Mapper>,
}

impl Bus {
  pub fn new(cartridge: Cartridge) -> Bus {
    let mapper = Mapper::new(cartridge);
    Bus {
      ram: [0; 2048],
      mapper: mapper,
    }
  }

  pub fn read_byte(&mut self, addr: u16) -> u8 {
    if addr < 0x2000 {
      self.ram[addr as usize % 0x0800]
    } else if addr < 0x4000 {
      panic!("read_byte in < 0x4000")
    } else if addr == 0x4016 {
      panic!("read_byte in == 0x4016")
    } else if addr <= 0x4018 {
      panic!("read_byte in == 0x4018")
    } else if addr < 0x6000 {
      0
    } else if addr > 0x6000 {
      self.mapper.load_byte(addr)
    } else {
      panic!("Erroneous read detected at {}", addr)
    }
  }

  pub fn read_word(&mut self, addr: u16) -> u16 {
    let lo = self.read_byte(addr);
    let hi = self.read_byte(addr + 1);
    util::make_word(lo, hi)
  }
}
