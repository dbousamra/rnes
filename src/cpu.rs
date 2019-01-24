use crate::bus::Bus;
use crate::opcode::*;
use crate::trace::Trace;
use crate::util;

enum Flag {
  Negative = 0,
  Overflow = 1,
  Unused = 2,
  Break = 3,
  Decimal = 4,
  InterruptDisable = 5,
  Zero = 6,
  Carry = 7,
}

pub struct Cpu {
  pub sp: u8,
  pub pc: u16,
  pub a: u8,
  pub x: u8,
  pub y: u8,
  pub p: u8,
  pub cycles: u64,
  pub bus: Bus,
}

impl Cpu {
  pub fn new(bus: Bus) -> Cpu {
    Cpu {
      pc: 0x0,
      sp: 0xFD,
      a: 0x0,
      x: 0x0,
      y: 0x0,
      p: 0x24,
      cycles: 0,
      bus: bus,
    }
  }

  pub fn reset(&mut self) -> () {
    self.pc = self.bus.read_word(0xFFFC);
    self.sp = 0xFD;
    self.p = 0x24;
  }

  pub fn step(&mut self, debug: bool) -> (Trace, u64) {
    let starting_cycles = self.cycles;

    let opcode = Opcode::new(self.bus.read_byte(self.pc));
    let trace = self.trace(opcode);
    if (debug) {
      println!("{}", trace);
    }

    let (page_crossed, address) = self.address_page_cross_for_mode(opcode.address_mode);

    self.pc += opcode.length as u16;
    self.cycles += Cpu::get_cycles(opcode, page_crossed) as u64;

    self.run_opcode(opcode, address);

    let ending_cycles = self.cycles;

    (trace, ending_cycles - starting_cycles)
  }

  fn address_page_cross_for_mode(&mut self, address_mode: AddressMode) -> (bool, u16) {
    match address_mode {
      AddressMode::Absolute => {
        let value = self.bus.read_word(self.pc + 1);
        let page_crossed = false;
        (page_crossed, value)
      }
      AddressMode::AbsoluteX => {
        let value = self.bus.read_word(self.pc + 1);
        let address_value = value.wrapping_add(self.x as u16);
        let page_crossed =
          Cpu::different_pages(address_value.wrapping_sub(self.x as u16), address_value);
        (page_crossed, address_value)
      }
      AddressMode::AbsoluteY => {
        let value = self.bus.read_word(self.pc + 1);
        let address_value = value.wrapping_add(self.y as u16);
        let page_crossed =
          Cpu::different_pages(address_value.wrapping_sub(self.y as u16), address_value);
        (page_crossed, address_value)
      }
      AddressMode::Accumulator => (false, 0),
      AddressMode::Immediate => (false, self.pc + 1),
      AddressMode::Implied => (false, 0),
      AddressMode::Indirect => {
        let value = self.bus.read_word(self.pc + 1);
        let address_value = self.read_word_bug(value);
        let page_crossed = false;
        (page_crossed, address_value)
      }
      AddressMode::IndirectIndexed => {
        let value = self.bus.read_byte(self.pc + 1);
        let address_value = self.read_word_bug(value as u16).wrapping_add(self.y as u16);
        let page_crossed =
          Cpu::different_pages(address_value.wrapping_sub(self.y as u16), address_value);
        (page_crossed, address_value)
      }
      AddressMode::IndexedIndirect => {
        let value = self.bus.read_byte(self.pc + 1);
        let address_value = self.read_word_bug(value.wrapping_add(self.x) as u16);
        let page_crossed = false;
        (page_crossed, address_value)
      }
      AddressMode::Relative => {
        let offset_16 = self.bus.read_word(self.pc + 1);
        let offset_8 = util::first_nibble(offset_16);
        let diff = if offset_8 < 0x80 { 0 } else { 0x100 };
        let page_crossed = false;
        (page_crossed, self.pc + 2 + offset_8 - diff)
      }
      AddressMode::ZeroPage => {
        let value = self.bus.read_byte(self.pc + 1) as u16;
        let page_crossed = false;
        (page_crossed, value)
      }

      AddressMode::ZeroPageX => {
        let value = (self.bus.read_byte(self.pc + 1).wrapping_add(self.x)) as u16;
        let page_crossed = false;
        (page_crossed, value)
      }
      AddressMode::ZeroPageY => {
        let value = (self.bus.read_byte(self.pc + 1).wrapping_add(self.y)) as u16;
        let page_crossed = false;
        (page_crossed, value)
      }
    }
  }

  fn different_pages(a: u16, b: u16) -> bool {
    (a & 0xFF00) != (b & 0xFF00)
  }

  fn get_cycles(opcode: Opcode, page_crossed: bool) -> u16 {
    if page_crossed {
      opcode.page_cross_cycles as u16 + opcode.cycles as u16
    } else {
      opcode.cycles as u16
    }
  }

  fn read_word_bug(&mut self, address: u16) -> u16 {
    let lo = self.bus.read_byte(address);
    let hi = self
      .bus
      .read_byte((address & 0xFF00) | (address + 1) as u8 as u16);
    util::make_word(lo, hi)
  }

  fn run_opcode(&mut self, opcode: Opcode, address: u16) -> () {
    match opcode.mnemonic {
      Mnemonic::ADC => self.adc(address),
      Mnemonic::AND => self.and(address),
      Mnemonic::ASL => self.asl(address, opcode.address_mode),
      Mnemonic::BCC => self.bcc(address),
      Mnemonic::BCS => self.bcs(address),
      Mnemonic::BEQ => self.beq(address),
      Mnemonic::BIT => self.bit(address),
      Mnemonic::BMI => self.bmi(address),
      Mnemonic::BNE => self.bne(address),
      Mnemonic::BPL => self.bpl(address),
      Mnemonic::BRK => self.brk(),
      Mnemonic::BVC => self.bvc(address),
      Mnemonic::BVS => self.bvs(address),
      Mnemonic::CLC => self.clc(),
      Mnemonic::CLD => self.cld(),
      // Mnemonic::CLI => self.cli(address),
      Mnemonic::CLV => self.clv(),
      Mnemonic::CMP => self.cmp(address),
      Mnemonic::CPX => self.cpx(address),
      Mnemonic::CPY => self.cpy(address),
      Mnemonic::DEC => self.dec(address),
      Mnemonic::DEX => self.dex(),
      Mnemonic::DEY => self.dey(),
      Mnemonic::EOR => self.eor(address),
      Mnemonic::INC => self.inc(address),
      Mnemonic::INX => self.inx(),
      Mnemonic::INY => self.iny(),
      Mnemonic::JMP => self.jmp(address),
      Mnemonic::JSR => self.jsr(address),
      Mnemonic::LDA => self.lda(address),
      Mnemonic::LDX => self.ldx(address),
      Mnemonic::LDY => self.ldy(address),
      Mnemonic::LSR => self.lsr(address, opcode.address_mode),
      Mnemonic::NOP => self.nop(),
      Mnemonic::PHA => self.pha(),
      Mnemonic::PHP => self.php(),
      Mnemonic::PLA => self.pla(),
      Mnemonic::PLP => self.plp(),
      Mnemonic::ORA => self.ora(address),
      Mnemonic::RTI => self.rti(),
      Mnemonic::RTS => self.rts(),
      Mnemonic::ROR => self.ror(address, opcode.address_mode),
      Mnemonic::ROL => self.rol(address, opcode.address_mode),
      Mnemonic::SBC => self.sbc(address),
      Mnemonic::SEC => self.sec(),
      Mnemonic::SED => self.sed(),
      Mnemonic::SEI => self.sei(),
      Mnemonic::STA => self.sta(address),
      Mnemonic::STX => self.stx(address),
      Mnemonic::STY => self.sty(address),
      Mnemonic::TAX => self.tax(),
      Mnemonic::TAY => self.tay(),
      Mnemonic::TSX => self.tsx(),
      Mnemonic::TXA => self.txa(),
      Mnemonic::TXS => self.txs(),
      Mnemonic::TYA => self.tya(),
      Mnemonic::KIL => self.illegal(opcode),
      Mnemonic::LAX => self.lax(address),
      Mnemonic::SAX => self.sax(address),
      Mnemonic::DCP => self.dcp(address),
      Mnemonic::ISC => self.isc(address),
      Mnemonic::RLA => self.rla(address, opcode.address_mode),
      Mnemonic::RRA => self.rra(address, opcode.address_mode),
      Mnemonic::SLO => self.slo(address, opcode.address_mode),
      Mnemonic::SRE => self.sre(address, opcode.address_mode),
      // Mnemonic::ANC => self.anc(address),
      // Mnemonic::ALR => self.alr(address),
      // Mnemonic::ARR => self.arr(address),
      // Mnemonic::XAA => self.xaa(address),
      // Mnemonic::AHX => self.ahx(address),
      // Mnemonic::TAS => self.tas(address),
      // Mnemonic::SHX => self.shx(address),
      // Mnemonic::SHY => self.shy(address),
      // Mnemonic::LAS => self.las(address),
      // Mnemonic::AXS => self.axs(address),
      _ => panic!("Opcode not implemented {}", opcode),
    }
  }

  fn adc(&mut self, address: u16) -> () {
    let a = self.a;
    let b = self.bus.read_byte(address);
    let c = self.get_flag(Flag::Carry) as u8;
    let result = a as u16 + b as u16 + c as u16;
    self.a = result as u8;
    self.set_zn(self.a);

    let should_carry = result > 0xFF;
    let does_overflow = ((a ^ b) & 0x80) == 0 && ((a ^ self.a) & 0x80) != 0;
    self.set_flag(Flag::Carry, should_carry);
    self.set_flag(Flag::Overflow, does_overflow);
  }

  fn and(&mut self, address: u16) -> () {
    let a_value = self.a;
    let value = self.bus.read_byte(address);
    self.a = a_value & value;
    self.set_zn(self.a)
  }

  fn asl(&mut self, address: u16, mode: AddressMode) -> () {
    let value = match mode {
      AddressMode::Accumulator => self.a,
      _ => self.bus.read_byte(address),
    };

    let result = value << 1;
    self.set_flag(Flag::Carry, value & 0b10000000 != 0);
    match mode {
      AddressMode::Accumulator => self.a = result,
      _ => self.bus.write_byte(address, result),
    }
    self.set_zn(result);
  }

  fn bcc(&mut self, address: u16) -> () {
    let flag = self.get_flag(Flag::Carry);
    self.branch(!flag, address)
  }

  fn bcs(&mut self, address: u16) -> () {
    let flag = self.get_flag(Flag::Carry);
    self.branch(flag, address)
  }

  fn beq(&mut self, address: u16) -> () {
    let flag = self.get_flag(Flag::Zero);
    self.branch(flag, address)
  }

  fn bit(&mut self, address: u16) -> () {
    let value = self.bus.read_byte(address);
    self.set_v(value);
    self.set_z(value & self.a);
    self.set_n(value)
  }

  fn bne(&mut self, address: u16) -> () {
    let flag = self.get_flag(Flag::Zero);
    self.branch(!flag, address)
  }

  fn bpl(&mut self, address: u16) -> () {
    let flag = self.get_flag(Flag::Negative);
    self.branch(!flag, address)
  }

  fn bmi(&mut self, address: u16) -> () {
    let flag = self.get_flag(Flag::Negative);
    self.branch(flag, address)
  }

  fn brk(&mut self) -> () {
    self.push_word(self.pc + 1);
    self.php();
    self.sei();
    self.pc = self.bus.read_word(0xFFFE);
  }

  fn bvc(&mut self, address: u16) -> () {
    let flag = self.get_flag(Flag::Overflow);
    self.branch(!flag, address)
  }

  fn bvs(&mut self, address: u16) -> () {
    let flag = self.get_flag(Flag::Overflow);
    self.branch(flag, address)
  }

  fn clc(&mut self) -> () {
    self.set_flag(Flag::Carry, false)
  }

  fn cld(&mut self) -> () {
    self.set_flag(Flag::Decimal, false)
  }

  // fn cli(&mut self, address: u16) -> () {
  //   panic!("Not implemented: cli")
  // }

  fn clv(&mut self) -> () {
    self.set_flag(Flag::Overflow, false)
  }

  fn cmp(&mut self, address: u16) -> () {
    let value = self.bus.read_byte(address);
    self.compare(self.a, value)
  }

  fn cpx(&mut self, address: u16) -> () {
    let value = self.bus.read_byte(address);
    self.compare(self.x, value)
  }

  fn cpy(&mut self, address: u16) -> () {
    let value = self.bus.read_byte(address);
    self.compare(self.y, value)
  }

  fn dec(&mut self, address: u16) -> () {
    let value = self.bus.read_byte(address).wrapping_sub(1);
    self.bus.write_byte(address, value);
    self.set_zn(value)
  }

  fn dex(&mut self) -> () {
    self.x = self.x.wrapping_sub(1);
    self.set_zn(self.x)
  }

  fn dey(&mut self) -> () {
    self.y = self.y.wrapping_sub(1);
    self.set_zn(self.y)
  }

  fn eor(&mut self, address: u16) -> () {
    let value = self.bus.read_byte(address);
    self.a = self.a ^ value;
    self.set_zn(self.a)
  }

  fn inc(&mut self, address: u16) -> () {
    let value = self.bus.read_byte(address).wrapping_add(1);
    self.bus.write_byte(address, value);
    self.set_zn(value)
  }

  fn inx(&mut self) -> () {
    self.x = self.x.wrapping_add(1);
    self.set_zn(self.x);
  }

  fn iny(&mut self) -> () {
    self.y = self.y.wrapping_add(1);
    self.set_zn(self.y);
  }

  fn jmp(&mut self, address: u16) -> () {
    self.pc = address;
  }

  fn jsr(&mut self, address: u16) -> () {
    self.push_word(self.pc - 1);
    self.pc = address;
  }

  fn lda(&mut self, address: u16) -> () {
    let value = self.bus.read_byte(address);
    self.a = value;
    self.set_zn(value);
  }

  fn ldx(&mut self, address: u16) -> () {
    let value = self.bus.read_byte(address);
    self.x = value;
    self.set_zn(value);
  }

  fn ldy(&mut self, address: u16) -> () {
    let value = self.bus.read_byte(address);
    self.y = value;
    self.set_zn(value);
  }

  fn lsr(&mut self, address: u16, mode: AddressMode) -> () {
    let value = match mode {
      AddressMode::Accumulator => self.a,
      _ => self.bus.read_byte(address),
    };

    let result = value >> 1;
    self.set_flag(Flag::Carry, value & 1 != 0);
    match mode {
      AddressMode::Accumulator => self.a = result,
      _ => self.bus.write_byte(address, result),
    }
    self.set_zn(result);
  }

  fn nop(&mut self) -> () {
    ()
  }

  fn pha(&mut self) -> () {
    self.push_byte(self.a)
  }

  fn php(&mut self) -> () {
    self.push_byte(self.p | 0x10)
  }

  fn pla(&mut self) -> () {
    let value = self.pull_byte();
    self.a = value;
    self.set_zn(value)
  }

  fn plp(&mut self) -> () {
    self.p = (self.pull_byte() & 0xEF) | 0x20
  }

  fn ora(&mut self, address: u16) -> () {
    let value = self.bus.read_byte(address);
    self.a = self.a | value;
    self.set_zn(self.a)
  }

  fn rti(&mut self) -> () {
    let address = self.pull_byte();
    self.p = address & 0xEF | 0x20;
    self.pc = self.pull_word();
  }

  fn rts(&mut self) -> () {
    let address = self.pull_word();
    self.pc = address + 1;
  }

  fn ror(&mut self, address: u16, mode: AddressMode) -> () {
    let value = match mode {
      AddressMode::Accumulator => self.a,
      _ => self.bus.read_byte(address),
    };

    let carry_value = self.get_flag(Flag::Carry) as u8;
    let result = (value >> 1) | (carry_value << 7);
    self.set_flag(Flag::Carry, value & 1 != 0);
    match mode {
      AddressMode::Accumulator => self.a = result,
      _ => self.bus.write_byte(address, result),
    }
    self.set_zn(result);
  }

  fn rol(&mut self, address: u16, mode: AddressMode) -> () {
    let value = match mode {
      AddressMode::Accumulator => self.a,
      _ => self.bus.read_byte(address),
    };

    let carry_value = self.get_flag(Flag::Carry) as u8;
    let result = (value << 1) | carry_value;
    self.set_flag(Flag::Carry, value & 0b10000000 != 0);
    match mode {
      AddressMode::Accumulator => self.a = result,
      _ => self.bus.write_byte(address, result),
    }
    self.set_zn(result);
  }

  fn sbc(&mut self, address: u16) -> () {
    let a = self.a;
    let b = self.bus.read_byte(address);
    let c = self.get_flag(Flag::Carry) as u8;
    let result = a as i16 - b as i16 - (1 - c as i16);
    self.a = result as u8;
    self.set_zn(self.a);

    let should_carry = result >= 0;
    let does_overflow = ((a ^ b) & 0x80) != 0 && ((a ^ self.a) & 0x80) != 0;
    self.set_flag(Flag::Carry, should_carry);
    self.set_flag(Flag::Overflow, does_overflow);
  }

  fn sec(&mut self) -> () {
    self.set_flag(Flag::Carry, true)
  }

  fn sed(&mut self) -> () {
    self.set_flag(Flag::Decimal, true)
  }

  fn sei(&mut self) -> () {
    self.set_flag(Flag::InterruptDisable, true)
  }

  fn sta(&mut self, address: u16) -> () {
    self.bus.write_byte(address, self.a)
  }

  fn stx(&mut self, address: u16) -> () {
    self.bus.write_byte(address, self.x)
  }

  fn sty(&mut self, address: u16) -> () {
    self.bus.write_byte(address, self.y)
  }

  fn tax(&mut self) -> () {
    self.x = self.a;
    self.set_zn(self.x)
  }

  fn tay(&mut self) -> () {
    self.y = self.a;
    self.set_zn(self.y)
  }

  fn tsx(&mut self) -> () {
    self.x = self.sp;
    self.set_zn(self.x)
  }

  fn txa(&mut self) -> () {
    self.a = self.x;
    self.set_zn(self.a)
  }

  fn txs(&mut self) -> () {
    self.sp = self.x
  }

  fn tya(&mut self) -> () {
    self.a = self.y;
    self.set_zn(self.a)
  }

  // fn kil(&mut self, address: u16) -> () {
  //   panic!("Not implemented: kil")
  // }

  fn lax(&mut self, address: u16) -> () {
    let value = self.bus.read_byte(address);
    self.a = value;
    self.x = value;
    self.set_zn(value);
  }

  fn sax(&mut self, address: u16) -> () {
    self.bus.write_byte(address, self.a & self.x)
  }

  fn dcp(&mut self, address: u16) -> () {
    self.dec(address);
    self.cmp(address);
  }

  fn isc(&mut self, address: u16) -> () {
    self.inc(address);
    self.sbc(address);
  }

  fn rla(&mut self, address: u16, address_mode: AddressMode) -> () {
    self.rol(address, address_mode);
    self.and(address);
  }

  fn rra(&mut self, address: u16, address_mode: AddressMode) -> () {
    self.ror(address, address_mode);
    self.adc(address);
  }

  fn slo(&mut self, address: u16, address_mode: AddressMode) -> () {
    self.asl(address, address_mode);
    self.ora(address);
  }

  fn sre(&mut self, address: u16, address_mode: AddressMode) -> () {
    self.lsr(address, address_mode);
    self.eor(address);
  }

  // fn anc(&mut self, address: u16) -> () {
  //   panic!("Not implemented: anc")
  // }

  // fn alr(&mut self, address: u16) -> () {
  //   panic!("Not implemented: alr")
  // }

  // fn arr(&mut self, address: u16) -> () {
  //   panic!("Not implemented: arr")
  // }

  // fn xaa(&mut self, address: u16) -> () {
  //   panic!("Not implemented: xaa")
  // }

  // fn ahx(&mut self, address: u16) -> () {
  //   panic!("Not implemented: ahx")
  // }

  // fn tas(&mut self, address: u16) -> () {
  //   panic!("Not implemented: tas")
  // }

  // fn shx(&mut self, address: u16) -> () {
  //   panic!("Not implemented: shx")
  // }

  // fn shy(&mut self, address: u16) -> () {
  //   panic!("Not implemented: shy")
  // }

  // fn las(&mut self, address: u16) -> () {
  //   panic!("Not implemented: las")
  // }

  // fn axs(&mut self, address: u16) -> () {
  //   panic!("Not implemented: axs")
  // }

  // push :: Word8 -> Emulator ()
  // push v = do
  //   spv <- loadCpu sp
  //   let i = 0x100 .|. toWord16 spv
  //   writeCpuMemory8 i v
  //   storeCpu sp (spv - 1)

  // push16 :: Word16 -> Emulator ()
  // push16 v = do
  //   let (lo, hi) = splitW16 v
  //   push hi
  //   push lo

  fn push_byte(&mut self, value: u8) -> () {
    let sp_value = self.sp;
    let i = 0x100 | sp_value as u16;
    self.bus.write_byte(i, value);
    self.sp = sp_value.wrapping_sub(1);
  }

  fn push_word(&mut self, value: u16) -> () {
    let (lo, hi) = util::split_word(value);
    self.push_byte(hi);
    self.push_byte(lo);
  }

  fn pull_byte(&mut self) -> u8 {
    let sp_value = self.sp;
    self.sp = sp_value.wrapping_add(1);
    let i = 0x100 | sp_value as u16 + 1;
    self.bus.read_byte(i)
  }

  fn pull_word(&mut self) -> u16 {
    let lo = self.pull_byte();
    let hi = self.pull_byte();
    util::make_word(lo, hi)
  }

  fn branch(&mut self, condition: bool, address: u16) -> () {
    if condition {
      let pc_value = self.pc;
      self.pc = address;
      let cycles = if Cpu::different_pages(pc_value, address) {
        2
      } else {
        1
      };
      self.cycles += cycles
    }
  }

  fn get_flag(&mut self, flag: Flag) -> bool {
    util::get_bit_at(self.p, 7 - flag as u8)
  }

  fn set_flag(&mut self, flag: Flag, condition: bool) -> () {
    let pos = 7 - flag as u8;
    if condition {
      self.p = util::set_bit_at(self.p, pos);
    } else {
      self.p = util::clear_bit_at(self.p, pos);
    }
  }

  fn set_z(&mut self, value: u8) -> () {
    self.set_flag(Flag::Zero, value == 0)
  }

  fn set_n(&mut self, value: u8) -> () {
    self.set_flag(Flag::Negative, value & 0x80 != 0)
  }

  fn set_v(&mut self, value: u8) -> () {
    self.set_flag(Flag::Overflow, value & 0x40 != 0)
  }

  fn set_zn(&mut self, value: u8) -> () {
    self.set_z(value);
    self.set_n(value);
  }

  fn compare(&mut self, a: u8, b: u8) -> () {
    let result = a as i16 - b as i16;
    self.set_zn(result as u8);
    self.set_flag(Flag::Carry, a >= b)
  }

  fn illegal(&mut self, opcode: Opcode) -> () {
    ()
  }

  fn trace(&mut self, opcode: Opcode) -> Trace {
    Trace::new(self, opcode)
  }
}
