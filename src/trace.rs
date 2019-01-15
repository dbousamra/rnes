use crate::cpu::Cpu;
use crate::opcode::Opcode;
use std::fmt;

pub struct Trace {
  pc: u16,
  sp: u8,
  a: u8,
  x: u8,
  y: u8,
  p: u8,
  op: Opcode,
  a0: u8,
  a1: u8,
  a2: u8,
  cyc: u64,
}

impl Trace {
  pub fn new(cpu: &mut Cpu, opcode: Opcode) -> Trace {
    let instrLength = opcode.length;
    let a0 = cpu.bus.read_byte(cpu.pc);
    let a1 = if instrLength < 2 {
      0x0
    } else {
      cpu.bus.read_byte(cpu.pc + 1)
    };
    let a2 = if instrLength < 3 {
      0x0
    } else {
      cpu.bus.read_byte(cpu.pc + 2)
    };

    Trace {
      pc: cpu.pc,
      sp: cpu.sp,
      a: cpu.a,
      x: cpu.x,
      y: cpu.y,
      p: cpu.p,
      op: opcode,
      a0: a0,
      a1: a1,
      a2: a2,
      cyc: cpu.cycles,
    }
  }
}

impl fmt::Display for Trace {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    write!(
      f,
      "{:04X}  {:02X}        {:?} {:27} A:{:02X} X:{:02X} Y:{:02X} P:{:02X} SP:{:02X} CYC:{:>3}",
      self.pc, self.a0, self.op.mnemonic, "", self.a, self.x, self.y, self.p, self.sp, self.cyc
    )
  }
}
