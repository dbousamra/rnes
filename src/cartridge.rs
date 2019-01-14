use std::fs::File;
use std::io::prelude::*;
use std::io::SeekFrom;

#[derive(Debug)]
pub struct INesHeader {
  pub magic: [u8; 4],
  pub prg_rom_size: u8,
  pub chr_rom_size: u8,
  pub flags_6: u8,
  pub flags_7: u8,
  pub prg_ram_size: u8,
  pub flags_9: u8,
  pub flags_10: u8,
  pub zero: [u8; 5],
}

impl INesHeader {
  pub fn mapper_id(&self) -> u8 {
    (self.flags_7 & 0xf0) | (self.flags_6 >> 4)
  }

  pub fn ines_mapper(&self) -> u8 {
    self.flags_6 >> 4
  }

  pub fn trainer(&self) -> bool {
    (self.flags_6 & 0x04) != 0
  }
}

#[derive(Debug)]
pub enum Mirror {
  MirrorHorizontal,
  MirrorVertical,
  MirrorSingle0,
  MirrorSingle1,
  MirrorFour,
}

#[derive(Debug)]
pub struct Cartridge {
  pub header: INesHeader,
  pub chr_rom: Vec<u8>,
  pub prg_rom: Vec<u8>,
  pub sram: Vec<u8>,
}

impl Cartridge {
  pub fn load_rom(path: String) -> Cartridge {
    let mut file = File::open(path).expect("ROM file not found");

    const HEADER_SIZE: usize = 0x10;
    const SRAM_SIZE: usize = 0x2000;
    let mut header = [0u8; HEADER_SIZE];

    file.read(&mut header).expect("Could not parse ROM header");

    let header = INesHeader {
      magic: [header[0], header[1], header[2], header[3]],
      prg_rom_size: header[4],
      chr_rom_size: header[5],
      flags_6: header[6],
      flags_7: header[7],
      prg_ram_size: header[8],
      flags_9: header[9],
      flags_10: header[10],
      zero: [0; 5],
    };

    if header.magic != *b"NES\x1a" {
      panic!("The ROM appears to be an an invalid format")
    }

    let prg_bytes = header.prg_rom_size as usize * 0x4000;
    let mut prg_rom = vec![0u8; prg_bytes];

    file
      .seek(SeekFrom::Start(HEADER_SIZE as u64))
      .expect("Not able to seek to PRG offset of ROM");

    file
      .read(&mut prg_rom)
      .expect("Could not read PRG bytes from ROM");

    let chr_bytes = header.chr_rom_size as usize * 0x2000;
    let mut chr_rom = vec![0u8; chr_bytes];

    file
      .seek(SeekFrom::Start(HEADER_SIZE as u64 + prg_bytes as u64))
      .expect("Not able to seek to CHR offset of ROM");

    file
      .read(&mut chr_rom)
      .expect("Could not read CHR bytes from ROM");

    let sram = vec![0u8; SRAM_SIZE];

    Cartridge {
      header: header,
      chr_rom: chr_rom,
      prg_rom: prg_rom,
      sram: sram,
    }
  }
}
