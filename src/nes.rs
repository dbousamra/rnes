use crate::cpu::Cpu;

pub struct Nes {
  cpu: Cpu,
}

impl Nes {
  pub fn new(cpu: Cpu) -> Nes {
    Nes { cpu: cpu }
  }

  pub fn reset(&mut self) -> () {
    self.cpu.reset()
  }

  pub fn step(&mut self) -> () {
    self.cpu.step();
  }

  pub fn run(&mut self) -> () {
    loop {
      self.step()
    }
  }
}
