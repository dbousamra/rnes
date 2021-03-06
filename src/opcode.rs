use std::fmt;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum AddressMode {
  Implied,
  Accumulator,
  Immediate,
  ZeroPage,
  ZeroPageX,
  ZeroPageY,
  Relative,
  Absolute,
  AbsoluteX,
  AbsoluteY,
  Indirect,
  IndexedIndirect,
  IndirectIndexed,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Mnemonic {
  // Official - 47
  ADC,
  AND,
  ASL,
  BCC,
  BCS,
  BEQ,
  BIT,
  BMI,
  BNE,
  BPL,
  BRK,
  BVC,
  BVS,
  CLC,
  CLD,
  CLI,
  CLV,
  CMP,
  CPX,
  CPY,
  DEC,
  DEX,
  DEY,
  EOR,
  INC,
  INX,
  INY,
  JMP,
  JSR,
  LDA,
  LDX,
  LDY,
  LSR,
  NOP,
  ORA,
  PHA,
  PHP,
  PLA,
  PLP,
  ROL,
  ROR,
  RTI,
  RTS,
  SBC,
  SEC,
  SED,
  SEI,
  STA,
  STX,
  STY,
  TAX,
  TAY,
  TSX,
  TXA,
  TXS,
  // Illegal / Unofficial - 19
  TYA,
  KIL,
  LAX,
  SAX,
  DCP,
  ISC,
  RLA,
  RRA,
  SLO,
  SRE,
  ANC,
  ALR,
  ARR,
  XAA,
  AHX,
  TAS,
  SHX,
  SHY,
  LAS,
  AXS,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Opcode {
  pub raw: u8,
  pub mnemonic: Mnemonic,
  pub address_mode: AddressMode,
  pub length: u8,
  pub cycles: u8,
  pub page_cross_cycles: u8,
}

impl Opcode {
  pub fn new(raw: u8) -> Opcode {
    let (mnemonic, address_mode, length, cycles, page_cross_cycles) = match raw {
      0x69 => (Mnemonic::ADC, AddressMode::Immediate, 2, 2, 0),
      0x65 => (Mnemonic::ADC, AddressMode::ZeroPage, 2, 3, 0),
      0x75 => (Mnemonic::ADC, AddressMode::ZeroPageX, 2, 4, 0),
      0x6D => (Mnemonic::ADC, AddressMode::Absolute, 3, 4, 0),
      0x7D => (Mnemonic::ADC, AddressMode::AbsoluteX, 3, 4, 1),
      0x79 => (Mnemonic::ADC, AddressMode::AbsoluteY, 3, 4, 1),
      0x61 => (Mnemonic::ADC, AddressMode::IndexedIndirect, 2, 6, 0),
      0x71 => (Mnemonic::ADC, AddressMode::IndirectIndexed, 2, 5, 1),
      0x29 => (Mnemonic::AND, AddressMode::Immediate, 2, 2, 0),
      0x25 => (Mnemonic::AND, AddressMode::ZeroPage, 2, 3, 0),
      0x35 => (Mnemonic::AND, AddressMode::ZeroPageX, 2, 4, 0),
      0x2D => (Mnemonic::AND, AddressMode::Absolute, 3, 4, 0),
      0x3D => (Mnemonic::AND, AddressMode::AbsoluteX, 3, 4, 1),
      0x39 => (Mnemonic::AND, AddressMode::AbsoluteY, 3, 4, 1),
      0x21 => (Mnemonic::AND, AddressMode::IndexedIndirect, 2, 6, 0),
      0x31 => (Mnemonic::AND, AddressMode::IndirectIndexed, 2, 5, 1),
      0x0A => (Mnemonic::ASL, AddressMode::Accumulator, 1, 2, 0),
      0x06 => (Mnemonic::ASL, AddressMode::ZeroPage, 2, 5, 0),
      0x16 => (Mnemonic::ASL, AddressMode::ZeroPageX, 2, 6, 0),
      0x0E => (Mnemonic::ASL, AddressMode::Absolute, 3, 6, 0),
      0x1E => (Mnemonic::ASL, AddressMode::AbsoluteX, 3, 7, 0),
      0x90 => (Mnemonic::BCC, AddressMode::Relative, 2, 2, 0),
      0xB0 => (Mnemonic::BCS, AddressMode::Relative, 2, 2, 0),
      0xF0 => (Mnemonic::BEQ, AddressMode::Relative, 2, 2, 1),
      0x24 => (Mnemonic::BIT, AddressMode::ZeroPage, 2, 3, 0),
      0x2C => (Mnemonic::BIT, AddressMode::Absolute, 3, 4, 0),
      0x30 => (Mnemonic::BMI, AddressMode::Relative, 2, 2, 0),
      0xD0 => (Mnemonic::BNE, AddressMode::Relative, 2, 2, 0),
      0x10 => (Mnemonic::BPL, AddressMode::Relative, 2, 2, 0),
      0x00 => (Mnemonic::BRK, AddressMode::Implied, 1, 7, 0),
      0x50 => (Mnemonic::BVC, AddressMode::Relative, 2, 2, 0),
      0x70 => (Mnemonic::BVS, AddressMode::Relative, 2, 2, 0),
      0x18 => (Mnemonic::CLC, AddressMode::Implied, 1, 2, 0),
      0xD8 => (Mnemonic::CLD, AddressMode::Implied, 1, 2, 0),
      0x58 => (Mnemonic::CLI, AddressMode::Implied, 1, 2, 0),
      0xB8 => (Mnemonic::CLV, AddressMode::Implied, 1, 2, 0),
      0xC9 => (Mnemonic::CMP, AddressMode::Immediate, 2, 2, 0),
      0xC5 => (Mnemonic::CMP, AddressMode::ZeroPage, 2, 3, 0),
      0xD5 => (Mnemonic::CMP, AddressMode::ZeroPageX, 2, 4, 0),
      0xCD => (Mnemonic::CMP, AddressMode::Absolute, 3, 4, 0),
      0xDD => (Mnemonic::CMP, AddressMode::AbsoluteX, 3, 4, 1),
      0xD9 => (Mnemonic::CMP, AddressMode::AbsoluteY, 3, 4, 1),
      0xC1 => (Mnemonic::CMP, AddressMode::IndexedIndirect, 2, 6, 0),
      0xD1 => (Mnemonic::CMP, AddressMode::IndirectIndexed, 2, 5, 1),
      0xE0 => (Mnemonic::CPX, AddressMode::Immediate, 2, 2, 0),
      0xE4 => (Mnemonic::CPX, AddressMode::ZeroPage, 2, 3, 0),
      0xEC => (Mnemonic::CPX, AddressMode::Absolute, 3, 4, 0),
      0xC0 => (Mnemonic::CPY, AddressMode::Immediate, 2, 2, 0),
      0xC4 => (Mnemonic::CPY, AddressMode::ZeroPage, 2, 3, 0),
      0xCC => (Mnemonic::CPY, AddressMode::Absolute, 3, 4, 0),
      0xC6 => (Mnemonic::DEC, AddressMode::ZeroPage, 2, 5, 0),
      0xD6 => (Mnemonic::DEC, AddressMode::ZeroPageX, 2, 6, 0),
      0xCE => (Mnemonic::DEC, AddressMode::Absolute, 3, 6, 0),
      0xDE => (Mnemonic::DEC, AddressMode::AbsoluteX, 3, 7, 0),
      0xCA => (Mnemonic::DEX, AddressMode::Implied, 1, 2, 0),
      0x88 => (Mnemonic::DEY, AddressMode::Implied, 1, 2, 0),
      0x49 => (Mnemonic::EOR, AddressMode::Immediate, 2, 2, 0),
      0x45 => (Mnemonic::EOR, AddressMode::ZeroPage, 2, 3, 0),
      0x55 => (Mnemonic::EOR, AddressMode::ZeroPageX, 2, 4, 0),
      0x4D => (Mnemonic::EOR, AddressMode::Absolute, 3, 4, 0),
      0x5D => (Mnemonic::EOR, AddressMode::AbsoluteX, 3, 4, 1),
      0x59 => (Mnemonic::EOR, AddressMode::AbsoluteY, 3, 4, 1),
      0x41 => (Mnemonic::EOR, AddressMode::IndexedIndirect, 2, 6, 0),
      0x51 => (Mnemonic::EOR, AddressMode::IndirectIndexed, 2, 5, 1),
      0xE6 => (Mnemonic::INC, AddressMode::ZeroPage, 2, 5, 0),
      0xF6 => (Mnemonic::INC, AddressMode::ZeroPageX, 2, 6, 0),
      0xEE => (Mnemonic::INC, AddressMode::Absolute, 3, 6, 0),
      0xFE => (Mnemonic::INC, AddressMode::AbsoluteX, 3, 7, 0),
      0xE8 => (Mnemonic::INX, AddressMode::Implied, 1, 2, 0),
      0xC8 => (Mnemonic::INY, AddressMode::Implied, 1, 2, 0),
      0x4C => (Mnemonic::JMP, AddressMode::Absolute, 3, 3, 0),
      0x6C => (Mnemonic::JMP, AddressMode::Indirect, 3, 5, 0),
      0x20 => (Mnemonic::JSR, AddressMode::Absolute, 3, 6, 0),
      0xA9 => (Mnemonic::LDA, AddressMode::Immediate, 2, 2, 0),
      0xA5 => (Mnemonic::LDA, AddressMode::ZeroPage, 2, 3, 0),
      0xB5 => (Mnemonic::LDA, AddressMode::ZeroPageX, 2, 4, 0),
      0xAD => (Mnemonic::LDA, AddressMode::Absolute, 3, 4, 0),
      0xBD => (Mnemonic::LDA, AddressMode::AbsoluteX, 3, 4, 1),
      0xB9 => (Mnemonic::LDA, AddressMode::AbsoluteY, 3, 4, 1),
      0xA1 => (Mnemonic::LDA, AddressMode::IndexedIndirect, 2, 6, 0),
      0xB1 => (Mnemonic::LDA, AddressMode::IndirectIndexed, 2, 5, 1),
      0xA2 => (Mnemonic::LDX, AddressMode::Immediate, 2, 2, 0),
      0xA6 => (Mnemonic::LDX, AddressMode::ZeroPage, 2, 3, 0),
      0xB6 => (Mnemonic::LDX, AddressMode::ZeroPageY, 2, 4, 0),
      0xAE => (Mnemonic::LDX, AddressMode::Absolute, 3, 4, 0),
      0xBE => (Mnemonic::LDX, AddressMode::AbsoluteY, 3, 4, 1),
      0xA0 => (Mnemonic::LDY, AddressMode::Immediate, 2, 2, 0),
      0xA4 => (Mnemonic::LDY, AddressMode::ZeroPage, 2, 3, 0),
      0xB4 => (Mnemonic::LDY, AddressMode::ZeroPageX, 2, 4, 0),
      0xAC => (Mnemonic::LDY, AddressMode::Absolute, 3, 4, 0),
      0xBC => (Mnemonic::LDY, AddressMode::AbsoluteX, 3, 4, 1),
      0x4A => (Mnemonic::LSR, AddressMode::Accumulator, 1, 2, 0),
      0x46 => (Mnemonic::LSR, AddressMode::ZeroPage, 2, 5, 0),
      0x56 => (Mnemonic::LSR, AddressMode::ZeroPageX, 2, 6, 0),
      0x4E => (Mnemonic::LSR, AddressMode::Absolute, 3, 6, 0),
      0x5E => (Mnemonic::LSR, AddressMode::AbsoluteX, 3, 7, 0),
      0xEA => (Mnemonic::NOP, AddressMode::Implied, 1, 2, 0),
      0x09 => (Mnemonic::ORA, AddressMode::Immediate, 2, 2, 0),
      0x05 => (Mnemonic::ORA, AddressMode::ZeroPage, 2, 3, 0),
      0x15 => (Mnemonic::ORA, AddressMode::ZeroPageX, 2, 4, 0),
      0x0D => (Mnemonic::ORA, AddressMode::Absolute, 3, 4, 0),
      0x1D => (Mnemonic::ORA, AddressMode::AbsoluteX, 3, 4, 1),
      0x19 => (Mnemonic::ORA, AddressMode::AbsoluteY, 3, 4, 1),
      0x01 => (Mnemonic::ORA, AddressMode::IndexedIndirect, 2, 6, 0),
      0x11 => (Mnemonic::ORA, AddressMode::IndirectIndexed, 2, 5, 1),
      0x48 => (Mnemonic::PHA, AddressMode::Implied, 1, 3, 0),
      0x08 => (Mnemonic::PHP, AddressMode::Implied, 1, 3, 0),
      0x68 => (Mnemonic::PLA, AddressMode::Implied, 1, 4, 0),
      0x28 => (Mnemonic::PLP, AddressMode::Implied, 1, 4, 0),
      0x2A => (Mnemonic::ROL, AddressMode::Accumulator, 1, 2, 0),
      0x26 => (Mnemonic::ROL, AddressMode::ZeroPage, 2, 5, 0),
      0x36 => (Mnemonic::ROL, AddressMode::ZeroPageX, 2, 6, 0),
      0x2E => (Mnemonic::ROL, AddressMode::Absolute, 3, 6, 0),
      0x3E => (Mnemonic::ROL, AddressMode::AbsoluteX, 3, 7, 0),
      0x6A => (Mnemonic::ROR, AddressMode::Accumulator, 1, 2, 0),
      0x66 => (Mnemonic::ROR, AddressMode::ZeroPage, 2, 5, 0),
      0x76 => (Mnemonic::ROR, AddressMode::ZeroPageX, 2, 6, 0),
      0x6E => (Mnemonic::ROR, AddressMode::Absolute, 3, 6, 0),
      0x7E => (Mnemonic::ROR, AddressMode::AbsoluteX, 3, 7, 0),
      0x40 => (Mnemonic::RTI, AddressMode::Implied, 1, 6, 0),
      0x60 => (Mnemonic::RTS, AddressMode::Implied, 1, 6, 0),
      0xE9 => (Mnemonic::SBC, AddressMode::Immediate, 2, 2, 0),
      0xE5 => (Mnemonic::SBC, AddressMode::ZeroPage, 2, 3, 0),
      0xF5 => (Mnemonic::SBC, AddressMode::ZeroPageX, 2, 4, 0),
      0xED => (Mnemonic::SBC, AddressMode::Absolute, 3, 4, 0),
      0xFD => (Mnemonic::SBC, AddressMode::AbsoluteX, 3, 4, 1),
      0xF9 => (Mnemonic::SBC, AddressMode::AbsoluteY, 3, 4, 1),
      0xE1 => (Mnemonic::SBC, AddressMode::IndexedIndirect, 2, 6, 0),
      0xF1 => (Mnemonic::SBC, AddressMode::IndirectIndexed, 2, 5, 1),
      0x38 => (Mnemonic::SEC, AddressMode::Implied, 1, 2, 0),
      0xF8 => (Mnemonic::SED, AddressMode::Implied, 1, 2, 0),
      0x78 => (Mnemonic::SEI, AddressMode::Implied, 1, 2, 0),
      0x85 => (Mnemonic::STA, AddressMode::ZeroPage, 2, 3, 0),
      0x95 => (Mnemonic::STA, AddressMode::ZeroPageX, 2, 4, 0),
      0x8D => (Mnemonic::STA, AddressMode::Absolute, 3, 4, 0),
      0x9D => (Mnemonic::STA, AddressMode::AbsoluteX, 3, 5, 0),
      0x99 => (Mnemonic::STA, AddressMode::AbsoluteY, 3, 5, 0),
      0x81 => (Mnemonic::STA, AddressMode::IndexedIndirect, 2, 6, 0),
      0x91 => (Mnemonic::STA, AddressMode::IndirectIndexed, 2, 6, 0),
      0x86 => (Mnemonic::STX, AddressMode::ZeroPage, 2, 3, 0),
      0x96 => (Mnemonic::STX, AddressMode::ZeroPageY, 2, 4, 0),
      0x8E => (Mnemonic::STX, AddressMode::Absolute, 3, 4, 0),
      0x84 => (Mnemonic::STY, AddressMode::ZeroPage, 2, 3, 0),
      0x94 => (Mnemonic::STY, AddressMode::ZeroPageX, 2, 4, 0),
      0x8C => (Mnemonic::STY, AddressMode::Absolute, 3, 4, 0),
      0xAA => (Mnemonic::TAX, AddressMode::Implied, 1, 2, 0),
      0xA8 => (Mnemonic::TAY, AddressMode::Implied, 1, 2, 0),
      0xBA => (Mnemonic::TSX, AddressMode::Implied, 1, 2, 0),
      0x8A => (Mnemonic::TXA, AddressMode::Implied, 1, 2, 0),
      0x9A => (Mnemonic::TXS, AddressMode::Implied, 1, 2, 0),
      0x98 => (Mnemonic::TYA, AddressMode::Implied, 1, 2, 0),

      // Illegal / Unofficial
      0x0B => (Mnemonic::ANC, AddressMode::Immediate, 2, 2, 0),
      0x2B => (Mnemonic::ANC, AddressMode::Immediate, 2, 2, 0),
      0x87 => (Mnemonic::SAX, AddressMode::ZeroPage, 2, 3, 0),
      0x97 => (Mnemonic::SAX, AddressMode::ZeroPageY, 2, 4, 0),
      0x83 => (Mnemonic::SAX, AddressMode::IndexedIndirect, 2, 6, 0),
      0x8F => (Mnemonic::SAX, AddressMode::Absolute, 3, 4, 0),
      0x6B => (Mnemonic::ARR, AddressMode::Immediate, 2, 2, 0),
      0x4B => (Mnemonic::ALR, AddressMode::Immediate, 2, 2, 0),
      0xAB => (Mnemonic::LAX, AddressMode::Immediate, 2, 2, 0),
      0x9F => (Mnemonic::AHX, AddressMode::AbsoluteY, 3, 5, 0),
      0x93 => (Mnemonic::AHX, AddressMode::IndirectIndexed, 2, 6, 0),
      0xCB => (Mnemonic::AXS, AddressMode::Immediate, 2, 2, 0),
      0xC7 => (Mnemonic::DCP, AddressMode::ZeroPage, 2, 5, 0),
      0xD7 => (Mnemonic::DCP, AddressMode::ZeroPageX, 2, 6, 0),
      0xCF => (Mnemonic::DCP, AddressMode::Absolute, 3, 6, 0),
      0xDF => (Mnemonic::DCP, AddressMode::AbsoluteX, 3, 7, 0),
      0xDB => (Mnemonic::DCP, AddressMode::AbsoluteY, 3, 7, 0),
      0xC3 => (Mnemonic::DCP, AddressMode::IndexedIndirect, 2, 8, 0),
      0x14 => (Mnemonic::NOP, AddressMode::ZeroPageX, 2, 4, 0),
      0x34 => (Mnemonic::NOP, AddressMode::ZeroPageX, 2, 4, 0),
      0x44 => (Mnemonic::NOP, AddressMode::ZeroPage, 2, 3, 0),
      0x54 => (Mnemonic::NOP, AddressMode::ZeroPageX, 2, 4, 0),
      0x64 => (Mnemonic::NOP, AddressMode::ZeroPage, 2, 3, 0),
      0x74 => (Mnemonic::NOP, AddressMode::ZeroPageX, 2, 4, 0),
      0x80 => (Mnemonic::NOP, AddressMode::Immediate, 2, 2, 0),
      0x82 => (Mnemonic::NOP, AddressMode::Immediate, 2, 2, 0),
      0x89 => (Mnemonic::NOP, AddressMode::Immediate, 2, 2, 0),
      0xC2 => (Mnemonic::NOP, AddressMode::Immediate, 2, 2, 0),
      0xD4 => (Mnemonic::NOP, AddressMode::ZeroPageX, 2, 4, 0),
      0xE2 => (Mnemonic::NOP, AddressMode::Immediate, 2, 2, 0),
      0xF4 => (Mnemonic::NOP, AddressMode::ZeroPageX, 2, 4, 0),
      0xE7 => (Mnemonic::ISC, AddressMode::ZeroPage, 2, 5, 0),
      0xF7 => (Mnemonic::ISC, AddressMode::ZeroPageX, 2, 6, 0),
      0xEF => (Mnemonic::ISC, AddressMode::Absolute, 3, 6, 0),
      0xFF => (Mnemonic::ISC, AddressMode::AbsoluteX, 3, 7, 0),
      0xFB => (Mnemonic::ISC, AddressMode::AbsoluteY, 3, 7, 0),
      0xE3 => (Mnemonic::ISC, AddressMode::IndexedIndirect, 2, 8, 0),
      0xF3 => (Mnemonic::ISC, AddressMode::IndirectIndexed, 2, 8, 0),
      0x02 => (Mnemonic::KIL, AddressMode::Implied, 1, 0, 0),
      0x12 => (Mnemonic::KIL, AddressMode::Implied, 1, 0, 0),
      0x22 => (Mnemonic::KIL, AddressMode::Implied, 1, 0, 0),
      0x32 => (Mnemonic::KIL, AddressMode::Implied, 1, 0, 0),
      0x42 => (Mnemonic::KIL, AddressMode::Implied, 1, 0, 0),
      0x52 => (Mnemonic::KIL, AddressMode::Implied, 1, 0, 0),
      0x62 => (Mnemonic::KIL, AddressMode::Implied, 1, 0, 0),
      0x72 => (Mnemonic::KIL, AddressMode::Implied, 1, 0, 0),
      0x92 => (Mnemonic::KIL, AddressMode::Implied, 1, 0, 0),
      0xB2 => (Mnemonic::KIL, AddressMode::Implied, 1, 0, 0),
      0xD2 => (Mnemonic::KIL, AddressMode::Implied, 1, 0, 0),
      0xF2 => (Mnemonic::KIL, AddressMode::Implied, 1, 0, 0),
      0xBB => (Mnemonic::LAS, AddressMode::AbsoluteY, 3, 4, 1),
      0xA7 => (Mnemonic::LAX, AddressMode::ZeroPage, 2, 3, 0),
      0xB7 => (Mnemonic::LAX, AddressMode::ZeroPageY, 2, 4, 0),
      0xAF => (Mnemonic::LAX, AddressMode::Absolute, 3, 4, 0),
      0xBF => (Mnemonic::LAX, AddressMode::AbsoluteY, 3, 4, 1),
      0xA3 => (Mnemonic::LAX, AddressMode::IndexedIndirect, 2, 6, 0),
      0xB3 => (Mnemonic::LAX, AddressMode::IndirectIndexed, 2, 5, 1),
      0x1A => (Mnemonic::NOP, AddressMode::Implied, 1, 2, 0),
      0x3A => (Mnemonic::NOP, AddressMode::Implied, 1, 2, 0),
      0x5A => (Mnemonic::NOP, AddressMode::Implied, 1, 2, 0),
      0x7A => (Mnemonic::NOP, AddressMode::Implied, 1, 2, 0),
      0xDA => (Mnemonic::NOP, AddressMode::Implied, 1, 2, 0),
      0xFA => (Mnemonic::NOP, AddressMode::Implied, 1, 2, 0),
      0x27 => (Mnemonic::RLA, AddressMode::ZeroPage, 2, 5, 0),
      0x37 => (Mnemonic::RLA, AddressMode::ZeroPageX, 2, 6, 0),
      0x2F => (Mnemonic::RLA, AddressMode::Absolute, 3, 6, 0),
      0x3F => (Mnemonic::RLA, AddressMode::AbsoluteX, 3, 7, 0),
      0x3B => (Mnemonic::RLA, AddressMode::AbsoluteY, 3, 7, 0),
      0x23 => (Mnemonic::RLA, AddressMode::IndexedIndirect, 2, 8, 0),
      0x33 => (Mnemonic::RLA, AddressMode::IndirectIndexed, 2, 8, 0),
      0x67 => (Mnemonic::RRA, AddressMode::ZeroPage, 2, 5, 0),
      0x77 => (Mnemonic::RRA, AddressMode::ZeroPageX, 2, 6, 0),
      0x6F => (Mnemonic::RRA, AddressMode::Absolute, 3, 6, 0),
      0x7F => (Mnemonic::RRA, AddressMode::AbsoluteX, 3, 7, 0),
      0x7B => (Mnemonic::RRA, AddressMode::AbsoluteY, 3, 7, 0),
      0x63 => (Mnemonic::RRA, AddressMode::IndexedIndirect, 2, 8, 0),
      0x73 => (Mnemonic::RRA, AddressMode::IndirectIndexed, 2, 8, 0),
      0xEB => (Mnemonic::SBC, AddressMode::Immediate, 2, 2, 0),
      0x07 => (Mnemonic::SLO, AddressMode::ZeroPage, 2, 5, 0),
      0x17 => (Mnemonic::SLO, AddressMode::ZeroPageX, 2, 6, 0),
      0x0F => (Mnemonic::SLO, AddressMode::Absolute, 3, 6, 0),
      0x1F => (Mnemonic::SLO, AddressMode::AbsoluteX, 3, 7, 0),
      0x1B => (Mnemonic::SLO, AddressMode::AbsoluteY, 3, 7, 0),
      0x03 => (Mnemonic::SLO, AddressMode::IndexedIndirect, 2, 8, 0),
      0x13 => (Mnemonic::SLO, AddressMode::IndirectIndexed, 2, 8, 0),
      0x47 => (Mnemonic::SRE, AddressMode::ZeroPage, 2, 5, 0),
      0x57 => (Mnemonic::SRE, AddressMode::ZeroPageX, 2, 6, 0),
      0x4F => (Mnemonic::SRE, AddressMode::Absolute, 3, 6, 0),
      0x5F => (Mnemonic::SRE, AddressMode::AbsoluteX, 3, 7, 0),
      0x5B => (Mnemonic::SRE, AddressMode::AbsoluteY, 3, 7, 0),
      0x43 => (Mnemonic::SRE, AddressMode::IndexedIndirect, 2, 8, 0),
      0x53 => (Mnemonic::SRE, AddressMode::IndirectIndexed, 2, 8, 0),
      0x9E => (Mnemonic::SHX, AddressMode::AbsoluteY, 3, 5, 0),
      0x9C => (Mnemonic::SHY, AddressMode::AbsoluteX, 3, 5, 0),
      0x0C => (Mnemonic::NOP, AddressMode::Absolute, 3, 4, 0),
      0x1C => (Mnemonic::NOP, AddressMode::AbsoluteX, 3, 4, 1),
      0x3C => (Mnemonic::NOP, AddressMode::AbsoluteX, 3, 4, 1),
      0x5C => (Mnemonic::NOP, AddressMode::AbsoluteX, 3, 4, 1),
      0x7C => (Mnemonic::NOP, AddressMode::AbsoluteX, 3, 4, 1),
      0xDC => (Mnemonic::NOP, AddressMode::AbsoluteX, 3, 4, 1),
      0xFC => (Mnemonic::NOP, AddressMode::AbsoluteX, 3, 4, 1),
      0xD3 => (Mnemonic::DCP, AddressMode::IndirectIndexed, 2, 8, 0),
      0x04 => (Mnemonic::NOP, AddressMode::ZeroPage, 2, 3, 0),
      0x8B => (Mnemonic::XAA, AddressMode::Immediate, 2, 2, 0),
      0x9B => (Mnemonic::TAS, AddressMode::AbsoluteY, 3, 5, 0),
      _ => panic!("0x{:X} is not a known opcode", raw),
    };

    Opcode {
      raw,
      mnemonic,
      address_mode,
      length,
      cycles,
      page_cross_cycles,
    }
  }
}

impl fmt::Display for Opcode {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    write!(
      f,
      "Opcode: ({}, {:?}, {:?}, {}, {}, {})",
      self.raw, self.mnemonic, self.address_mode, self.length, self.cycles, self.page_cross_cycles
    )
  }
}
