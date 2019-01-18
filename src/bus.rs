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

  pub fn read_byte(&mut self, address: u16) -> u8 {
    if address < 0x2000 {
      self.ram[address as usize % 0x0800]
    } else if address < 0x4000 {
      panic!("read_byte in < 0x4000")
    } else if address == 0x4016 {
      panic!("read_byte in == 0x4016")
    } else if address <= 0x4018 {
      panic!("read_byte in == 0x4018")
    } else if address < 0x6000 {
      0
    } else if address > 0x6000 {
      self.mapper.load_byte(address)
    } else {
      panic!("Erroneous read detected at {}", address)
    }
  }

  pub fn read_word(&mut self, address: u16) -> u16 {
    let lo = self.read_byte(address);
    let hi = self.read_byte(address + 1);
    util::make_word(lo, hi)
  }
}
