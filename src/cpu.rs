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

  pub fn step(&mut self) -> (u64, Trace) {
    let starting_cycles = self.cycles;

    let opcode = Opcode::new(self.bus.read_byte(self.pc));
    let trace = self.trace(opcode);

    let (page_crossed, address) = self.address_page_cross_for_mode(opcode.address_mode);
    self.pc += opcode.length as u16;
    self.run_opcode(opcode, address);

    let ending_cycles = self.cycles;
    let total_cycles_taken = ending_cycles - starting_cycles;

    (total_cycles_taken, trace)
  }

  fn address_page_cross_for_mode(&mut self, address_mode: AddressMode) -> (bool, u16) {
    match address_mode {
      AddressMode::Absolute => {
        let value = self.bus.read_word(self.pc);
        let page_crossed = false;
        (false, value)
      }
      AddressMode::AbsoluteX => {
        let value = self.bus.read_word(self.pc + 1);
        let address_value = value + self.x as u16;
        let page_crossed = Cpu::different_pages(address_value - self.x as u16, address_value);
        (page_crossed, address_value)
      }
      AddressMode::AbsoluteY => {
        let value = self.bus.read_word(self.pc + 1);
        let address_value = value + self.y as u16;
        let page_crossed = Cpu::different_pages(address_value - self.y as u16, address_value);
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
        let value = self.bus.read_byte(self.pc + 2);
        let address = self.read_word_bug(value as u16);
        let address_value = address + (self.y as u16);
        let page_crossed = Cpu::different_pages(address_value - self.y as u16, address_value);
        (page_crossed, address_value)
      }
      AddressMode::IndexedIndirect => {
        let value = self.bus.read_word(self.pc + 1);
        let address_value = self.read_word_bug(value + self.x as u16);
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
        let value = (self.bus.read_byte(self.pc + 1) + self.x) as u16;
        let page_crossed = false;
        (page_crossed, value)
      }
      AddressMode::ZeroPageY => {
        let value = (self.bus.read_byte(self.pc + 1) + self.y) as u16;
        let page_crossed = false;
        (page_crossed, value)
      }
    }
  }

  fn different_pages(a: u16, b: u16) -> bool {
    (a & 0xFF00) != (b & 0xFF00)
  }

  fn read_word_bug(&mut self, address: u16) -> u16 {
    let lo = self.bus.read_byte(address);
    let hi = self
      .bus
      .read_byte((address & 0xFF00) | (address + 1 as u8 as u16));
    util::make_word(lo, hi)
  }

  fn run_opcode(&mut self, opcode: Opcode, address: u16) -> () {
    match opcode.mnemonic {
      ADC => self.adc(address),
      // AND => self.and(address),
      // ASL => self.asl(address),
      // BCC => self.bcc(address),
      // BCS => self.bcs(address),
      // BEQ => self.beq(address),
      // BIT => self.bit(address),
      // BMI => self.bmi(address),
      // BNE => self.bne(address),
      // BPL => self.bpl(address),
      // BRK => self.brk(address),
      // BVC => self.bvc(address),
      // BVS => self.bvs(address),
      // CLC => self.clc(address),
      // CLD => self.cld(address),
      // CLI => self.cli(address),
      // CLV => self.clv(address),
      // CMP => self.cmp(address),
      // CPX => self.cpx(address),
      // CPY => self.cpy(address),
      // DEC => self.dec(address),
      // DEX => self.dex(address),
      // DEY => self.dey(address),
      // EOR => self.eor(address),
      // INC => self.inc(address),
      // INX => self.inx(address),
      // INY => self.iny(address),
      // JMP => self.jmp(address),
      // JSR => self.jsr(address),
      // LDA => self.lda(address),
      // LDX => self.ldx(address),
      // LDY => self.ldy(address),
      // LSR => self.lsr(address),
      // NOP => self.nop(address),
      // PHA => self.pha(address),
      // PHP => self.php(address),
      // PLA => self.pla(address),
      // PLP => self.plp(address),
      // ORA => self.ora(address),
      // RTI => self.rti(address),
      // RTS => self.rts(address),
      // ROR => self.ror(address),
      // ROL => self.rol(address),
      // SBC => self.sbc(address),
      // SEC => self.sec(address),
      // SED => self.sed(address),
      SEI => self.sei(address),
      // STA => self.sta(address),
      // STX => self.stx(address),
      // STY => self.sty(address),
      // TAX => self.tax(address),
      // TAY => self.tay(address),
      // TSX => self.tsx(address),
      // TXA => self.txa(address),
      // TXS => self.txs(address),
      // TYA => self.tya(address),
      // KIL => self.kil(address),
      // LAX => self.lax(address),
      // SAX => self.sax(address),
      // DCP => self.dcp(address),
      // ISC => self.isc(address),
      // RLA => self.rla(address),
      // RRA => self.rra(address),
      // SLO => self.slo(address),
      // SRE => self.sre(address),
      // ANC => self.anc(address),
      // ALR => self.alr(address),
      // ARR => self.arr(address),
      // XAA => self.xaa(address),
      // AHX => self.ahx(address),
      // TAS => self.tas(address),
      // SHX => self.shx(address),
      // SHY => self.shy(address),
      // LAS => self.las(address),
      // AXS => self.axs(address),
    }
  }

  fn adc(&mut self, address: u16) -> () {
    let a_value = self.a;
    let address_value = self.bus.read_byte(address);
    let carry_value = self.get_flag(Flag::Carry) as u8;
    println!("{}, {}, {}", a_value, address_value, carry_value);
    self.a = a_value + address_value + carry_value;
    let a_value_post = self.a;
    self.set_zn(a_value_post);
    let should_carry = (a_value as u32 + address_value as u32 + carry_value as u32) > 0xFF;
    let does_overflow =
      ((a_value ^ address_value) & 0x80) == 0 && ((a_value ^ a_value_post) & 0x80) != 0;

    self.set_flag(Flag::Carry, should_carry);
    self.set_flag(Flag::Overflow, does_overflow);
  }

  // fn and(&mut self, address: u16) -> () {
  //   panic!("Not implemented: and")
  // }

  // fn asl(&mut self, address: u16) -> () {
  //   panic!("Not implemented: asl")
  // }

  // fn bcc(&mut self, address: u16) -> () {
  //   panic!("Not implemented: bcc")
  // }

  // fn bcs(&mut self, address: u16) -> () {
  //   panic!("Not implemented: bcs")
  // }

  // fn beq(&mut self, address: u16) -> () {
  //   panic!("Not implemented: beq")
  // }

  // fn bit(&mut self, address: u16) -> () {
  //   panic!("Not implemented: bit")
  // }

  // fn bmi(&mut self, address: u16) -> () {
  //   panic!("Not implemented: bmi")
  // }

  // fn bne(&mut self, address: u16) -> () {
  //   panic!("Not implemented: bne")
  // }

  // fn bpl(&mut self, address: u16) -> () {
  //   panic!("Not implemented: bpl")
  // }

  // fn brk(&mut self, address: u16) -> () {
  //   panic!("Not implemented: brk")
  // }

  // fn bvc(&mut self, address: u16) -> () {
  //   panic!("Not implemented: bvc")
  // }

  // fn bvs(&mut self, address: u16) -> () {
  //   panic!("Not implemented: bvs")
  // }

  // fn clc(&mut self, address: u16) -> () {
  //   panic!("Not implemented: clc")
  // }

  // fn cld(&mut self, address: u16) -> () {
  //   panic!("Not implemented: cld")
  // }

  // fn cli(&mut self, address: u16) -> () {
  //   panic!("Not implemented: cli")
  // }

  // fn clv(&mut self, address: u16) -> () {
  //   panic!("Not implemented: clv")
  // }

  // fn cmp(&mut self, address: u16) -> () {
  //   panic!("Not implemented: cmp")
  // }

  // fn cpx(&mut self, address: u16) -> () {
  //   panic!("Not implemented: cpx")
  // }

  // fn cpy(&mut self, address: u16) -> () {
  //   panic!("Not implemented: cpy")
  // }

  // fn dec(&mut self, address: u16) -> () {
  //   panic!("Not implemented: dec")
  // }

  // fn dex(&mut self, address: u16) -> () {
  //   panic!("Not implemented: dex")
  // }

  // fn dey(&mut self, address: u16) -> () {
  //   panic!("Not implemented: dey")
  // }

  // fn eor(&mut self, address: u16) -> () {
  //   panic!("Not implemented: eor")
  // }

  // fn inc(&mut self, address: u16) -> () {
  //   panic!("Not implemented: inc")
  // }

  // fn inx(&mut self, address: u16) -> () {
  //   panic!("Not implemented: inx")
  // }

  // fn iny(&mut self, address: u16) -> () {
  //   panic!("Not implemented: iny")
  // }

  // fn jmp(&mut self, address: u16) -> () {
  //   panic!("Not implemented: jmp")
  // }

  // fn jsr(&mut self, address: u16) -> () {
  //   panic!("Not implemented: jsr")
  // }

  // fn lda(&mut self, address: u16) -> () {
  //   panic!("Not implemented: lda")
  // }

  // fn ldx(&mut self, address: u16) -> () {
  //   panic!("Not implemented: ldx")
  // }

  // fn ldy(&mut self, address: u16) -> () {
  //   panic!("Not implemented: ldy")
  // }

  // fn lsr(&mut self, address: u16) -> () {
  //   panic!("Not implemented: lsr")
  // }

  // fn nop(&mut self, address: u16) -> () {
  //   panic!("Not implemented: nop")
  // }

  // fn pha(&mut self, address: u16) -> () {
  //   panic!("Not implemented: pha")
  // }

  // fn php(&mut self, address: u16) -> () {
  //   panic!("Not implemented: php")
  // }

  // fn pla(&mut self, address: u16) -> () {
  //   panic!("Not implemented: pla")
  // }

  // fn plp(&mut self, address: u16) -> () {
  //   panic!("Not implemented: plp")
  // }

  // fn ora(&mut self, address: u16) -> () {
  //   panic!("Not implemented: ora")
  // }

  // fn rti(&mut self, address: u16) -> () {
  //   panic!("Not implemented: rti")
  // }

  // fn rts(&mut self, address: u16) -> () {
  //   panic!("Not implemented: rts")
  // }

  // fn ror(&mut self, address: u16) -> () {
  //   panic!("Not implemented: ror")
  // }

  // fn rol(&mut self, address: u16) -> () {
  //   panic!("Not implemented: rol")
  // }

  // fn sbc(&mut self, address: u16) -> () {
  //   panic!("Not implemented: sbc")
  // }

  // fn sec(&mut self, address: u16) -> () {
  //   panic!("Not implemented: sec")
  // }

  // fn sed(&mut self, address: u16) -> () {
  //   panic!("Not implemented: sed")
  // }

  fn sei(&mut self, address: u16) -> () {
    self.set_flag(Flag::InterruptDisable, true)
  }

  // fn sta(&mut self, address: u16) -> () {
  //   panic!("Not implemented: sta")
  // }

  // fn stx(&mut self, address: u16) -> () {
  //   panic!("Not implemented: stx")
  // }

  // fn sty(&mut self, address: u16) -> () {
  //   panic!("Not implemented: sty")
  // }

  // fn tax(&mut self, address: u16) -> () {
  //   panic!("Not implemented: tax")
  // }

  // fn tay(&mut self, address: u16) -> () {
  //   panic!("Not implemented: tay")
  // }

  // fn tsx(&mut self, address: u16) -> () {
  //   panic!("Not implemented: tsx")
  // }

  // fn txa(&mut self, address: u16) -> () {
  //   panic!("Not implemented: txa")
  // }

  // fn txs(&mut self, address: u16) -> () {
  //   panic!("Not implemented: txs")
  // }

  // fn tya(&mut self, address: u16) -> () {
  //   panic!("Not implemented: tya")
  // }

  // fn kil(&mut self, address: u16) -> () {
  //   panic!("Not implemented: kil")
  // }

  // fn lax(&mut self, address: u16) -> () {
  //   panic!("Not implemented: lax")
  // }

  // fn sax(&mut self, address: u16) -> () {
  //   panic!("Not implemented: sax")
  // }

  // fn dcp(&mut self, address: u16) -> () {
  //   panic!("Not implemented: dcp")
  // }

  // fn isc(&mut self, address: u16) -> () {
  //   panic!("Not implemented: isc")
  // }

  // fn rla(&mut self, address: u16) -> () {
  //   panic!("Not implemented: rla")
  // }

  // fn rra(&mut self, address: u16) -> () {
  //   panic!("Not implemented: rra")
  // }

  // fn slo(&mut self, address: u16) -> () {
  //   panic!("Not implemented: slo")
  // }

  // fn sre(&mut self, address: u16) -> () {
  //   panic!("Not implemented: sre")
  // }

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

  fn get_flag(&mut self, flag: Flag) -> bool {
    util::get_bit_at(self.p, 7 - flag as u8)
  }

  fn set_flag(&mut self, flag: Flag, value: bool) -> () {
    util::set_bit_at(self.p, 7 - flag as u8);
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

  fn trace(&mut self, opcode: Opcode) -> Trace {
    Trace::new(self, opcode)
  }

  fn not_implemented(&mut self, opcode: Opcode) -> () {
    panic!("Opcode not implemented: {}", opcode)
  }
}
