use crate::bus::Bus;
use crate::opcode::Opcode;

pub struct Cpu {
  pc: u16,
  sp: u8,
  a: u8,
  x: u8,
  y: u8,
  p: u8,
  cycles: u64,
  bus: Bus,
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

  pub fn step(&mut self) -> u64 {
    let starting_cycles = self.cycles;
    let opcode = Opcode::new(self.bus.read_byte(self.pc));
    println!("{:?}", opcode);
    self.pc += opcode.length as u16;
    let ending_cycles = self.cycles;
    ending_cycles - starting_cycles
  }
}
