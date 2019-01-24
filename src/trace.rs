use crate::cpu::Cpu;
use crate::opcode::Opcode;
use std::fmt;

#[derive(Debug, PartialEq, Eq)]
pub struct Trace {
  pc: u16,
  sp: u8,
  a: u8,
  x: u8,
  y: u8,
  p: u8,
  opcode: Opcode,
  a0: u8,
  cyc: u64,
}

impl Trace {
  pub fn new(cpu: &mut Cpu, opcode: Opcode) -> Trace {
    let a0 = cpu.bus.read_byte(cpu.pc);
    let cyc = (cpu.cycles * 3) % 341;

    Trace {
      pc: cpu.pc,
      sp: cpu.sp,
      a: cpu.a,
      x: cpu.x,
      y: cpu.y,
      p: cpu.p,
      opcode: opcode,
      a0: a0,
      cyc: cyc,
    }
  }

  pub fn from_string(input: String) -> Trace {
    let pc = u16::from_str_radix(&input[0..4].trim(), 16).unwrap();
    let a0 = u8::from_str_radix(&input[6..8].trim(), 16).unwrap();
    let a = u8::from_str_radix(&input[50..52].trim(), 16).unwrap();
    let x = u8::from_str_radix(&input[55..57].trim(), 16).unwrap();
    let y = u8::from_str_radix(&input[60..62].trim(), 16).unwrap();
    let p = u8::from_str_radix(&input[65..67].trim(), 16).unwrap();
    let sp = u8::from_str_radix(&input[71..73].trim(), 16).unwrap();

    let raw_cyc = &input[78..82].trim();

    let cyc = u64::from_str_radix(raw_cyc, 10).unwrap();
    let opcode = Opcode::new(a0);

    Trace {
      pc,
      sp,
      a,
      x,
      y,
      p,
      opcode,
      a0,
      cyc,
    }
  }
}

impl fmt::Display for Trace {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    write!(
      f,
      "{:04X}  {:02X}        {:?} {:27} A:{:02X} X:{:02X} Y:{:02X} P:{:02X} SP:{:02X} CYC:{:>3}",
      self.pc, self.a0, self.opcode.mnemonic, "", self.a, self.x, self.y, self.p, self.sp, self.cyc
    )
  }
}
