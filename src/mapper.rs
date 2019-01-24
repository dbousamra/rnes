use crate::cartridge::Cartridge;

pub trait Mapper {
  fn load_byte(&mut self, addr: u16) -> u8;

  fn write_byte(&mut self, addr: u16, value: u8);
}

impl Mapper {
  pub fn new(cartridge: Cartridge) -> Box<Mapper> {
    let prg_banks = (cartridge.prg_rom.len() / 0x4000) as u32;
    let prg_bank_1 = 0;
    let prg_bank_2 = prg_banks - 1 as u32;

    Box::new(Mapper2 {
      cartridge,
      prg_banks,
      prg_bank_1,
      prg_bank_2,
    })
  }
}

pub struct Mapper2 {
  cartridge: Cartridge,
  prg_banks: u32,
  prg_bank_1: u32,
  prg_bank_2: u32,
}

impl Mapper for Mapper2 {
  fn load_byte(&mut self, addr: u16) -> u8 {
    if addr < 0x2000 {
      let index = addr as usize;
      self.cartridge.chr_rom[index]
    } else if addr >= 0xC000 {
      let index = self.prg_bank_2 as usize * 0x4000 + (addr as usize - 0xC000);
      self.cartridge.prg_rom[index]
    } else if addr >= 0x8000 {
      let index = self.prg_bank_1 as usize * 0x4000 + (addr as usize - 0x8000);
      self.cartridge.prg_rom[index]
    } else if addr >= 0x6000 {
      let index = addr as usize - 0x6000;
      self.cartridge.sram[index]
    } else {
      panic!("Erroneous Mapper2 read detected at {}", addr)
    }
  }

  fn write_byte(&mut self, addr: u16, value: u8) {
    if addr < 0x2000 {
      let index = addr as usize;
      self.cartridge.chr_rom[index] = value;
    } else if addr >= 0x8000 {
      self.prg_bank_1 = value as u32 % self.prg_banks;
    } else if addr >= 0x6000 {
      let index = addr as usize - 0x6000;
      self.cartridge.sram[index] = value;
    } else {
      panic!("Erroneous Mapper2 write detected at {}", addr)
    }
  }
}
