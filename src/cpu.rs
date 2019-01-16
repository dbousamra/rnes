use crate::bus::Bus;
use crate::opcode::*;
use crate::trace::Trace;
use crate::util;

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
      ADC => {
        // adc
        self.not_implemented(opcode);
      }
      AND => {
        // and
        self.not_implemented(opcode);
      }
      ASL => {
        // asl mode
        self.not_implemented(opcode);
      }
      BCC => {
        // bcc
        self.not_implemented(opcode);
      }
      BCS => {
        // bcs
        self.not_implemented(opcode);
      }
      BEQ => {
        // beq
        self.not_implemented(opcode);
      }
      BIT => {
        // bit
        self.not_implemented(opcode);
      }
      BMI => {
        // bmi
        self.not_implemented(opcode);
      }
      BNE => {
        // bne
        self.not_implemented(opcode);
      }
      BPL => {
        // bpl
        self.not_implemented(opcode);
      }
      BRK => {
        // const brk
        self.not_implemented(opcode);
      }
      BVC => {
        // bvc
        self.not_implemented(opcode);
      }
      BVS => {
        // bvs
        self.not_implemented(opcode);
      }
      CLC => {
        // const clc
        self.not_implemented(opcode);
      }
      CLD => {
        // const cld
        self.not_implemented(opcode);
      }
      CLI => {
        // const cli
        self.not_implemented(opcode);
      }
      CLV => {
        // const clv
        self.not_implemented(opcode);
      }
      CMP => {
        // cmp
        self.not_implemented(opcode);
      }
      CPX => {
        // cpx
        self.not_implemented(opcode);
      }
      CPY => {
        // cpy
        self.not_implemented(opcode);
      }
      DEC => {
        // dec
        self.not_implemented(opcode);
      }
      DEX => {
        // const dex
        self.not_implemented(opcode);
      }
      DEY => {
        // const dey
        self.not_implemented(opcode);
      }
      EOR => {
        // eor
        self.not_implemented(opcode);
      }
      INC => {
        // inc
        self.not_implemented(opcode);
      }
      INX => {
        // const inx
        self.not_implemented(opcode);
      }
      INY => {
        // const iny
        self.not_implemented(opcode);
      }
      JMP => {
        // jmp
        self.not_implemented(opcode);
      }
      JSR => {
        // jsr
        self.not_implemented(opcode);
      }
      LDA => {
        // lda
        self.not_implemented(opcode);
      }
      LDX => {
        // ldx
        self.not_implemented(opcode);
      }
      LDY => {
        // ldy
        self.not_implemented(opcode);
      }
      LSR => {
        // lsr mode
        self.not_implemented(opcode);
      }
      NOP => {
        // const nop
        self.not_implemented(opcode);
      }
      PHA => {
        // const pha
        self.not_implemented(opcode);
      }
      PHP => {
        // const php
        self.not_implemented(opcode);
      }
      PLA => {
        // const pla
        self.not_implemented(opcode);
      }
      PLP => {
        // const plp
        self.not_implemented(opcode);
      }
      ORA => {
        // ora
        self.not_implemented(opcode);
      }
      RTI => {
        // const rti
        self.not_implemented(opcode);
      }
      RTS => {
        // const rts
        self.not_implemented(opcode);
      }
      ROR => {
        // ror mode
        self.not_implemented(opcode);
      }
      ROL => {
        // rol mode
        self.not_implemented(opcode);
      }
      SBC => {
        // sbc
        self.not_implemented(opcode);
      }
      SEC => {
        // const sec
        self.not_implemented(opcode);
      }
      SED => {
        // const sed
        self.not_implemented(opcode);
      }
      SEI => {
        // const sei
        self.not_implemented(opcode);
      }
      STA => {
        // sta
        self.not_implemented(opcode);
      }
      STX => {
        // stx
        self.not_implemented(opcode);
      }
      STY => {
        // sty
        self.not_implemented(opcode);
      }
      TAX => {
        // const tax
        self.not_implemented(opcode);
      }
      TAY => {
        // const tay
        self.not_implemented(opcode);
      }
      TSX => {
        // const tsx
        self.not_implemented(opcode);
      }
      TXA => {
        // const txa
        self.not_implemented(opcode);
      }
      TXS => {
        // const txs
        self.not_implemented(opcode);
      }
      TYA => {
        // const tya
        self.not_implemented(opcode);
      }
      KIL => {
        // const $ illegal mnemonic
        self.not_implemented(opcode);
      }
      LAX => {
        // lax
        self.not_implemented(opcode);
      }
      SAX => {
        // sax
        self.not_implemented(opcode);
      }
      DCP => {
        // dcp
        self.not_implemented(opcode);
      }
      ISC => {
        // isc
        self.not_implemented(opcode);
      }
      RLA => {
        // rla mode
        self.not_implemented(opcode);
      }
      RRA => {
        // rra mode
        self.not_implemented(opcode);
      }
      SLO => {
        // slo mode
        self.not_implemented(opcode);
      }
      SRE => {
        // sre mode
        self.not_implemented(opcode);
      }
      ANC => {
        // anc
        self.not_implemented(opcode);
      }
      ALR => {
        // alr
        self.not_implemented(opcode);
      }
      ARR => {
        // arr
        self.not_implemented(opcode);
      }
      XAA => {
        // const $ illegal mnemonic
        self.not_implemented(opcode);
      }
      AHX => {
        // const $ illegal mnemonic
        self.not_implemented(opcode);
      }
      TAS => {
        // const $ illegal mnemonic
        self.not_implemented(opcode);
      }
      SHX => {
        // shx
        self.not_implemented(opcode);
      }
      SHY => {
        // shy
        self.not_implemented(opcode);
      }
      LAS => {
        // const $ illegal mnemonic
        self.not_implemented(opcode);
      }
      AXS => {
        // axs
        self.not_implemented(opcode);
      }
    }
  }

  fn trace(&mut self, opcode: Opcode) -> Trace {
    Trace::new(self, opcode)
  }

  fn not_implemented(&mut self, opcode: Opcode) -> () {
    panic!("Opcode not implemented: {}", opcode)
  }
}
