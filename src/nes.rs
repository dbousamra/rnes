use crate::bus::Bus;
use crate::cartridge::Cartridge;
use crate::cpu::Cpu;
use crate::ppu::Ppu;
use crate::trace::Trace;

pub struct Nes {
  pub cpu: Cpu,
}

impl Nes {
  pub fn new(cpu: Cpu) -> Nes {
    Nes { cpu }
  }

  pub fn from_rom(path: String) -> Nes {
    let cartridge = Cartridge::load_rom(path);
    let ppu = Ppu::new();
    let bus = Bus::new(cartridge, ppu);
    let cpu = Cpu::new(bus);
    Nes::new(cpu)
  }

  pub fn reset(&mut self) -> () {
    self.cpu.reset();
    self.cpu.bus.ppu.reset();
  }

  pub fn step(&mut self, debug: bool) -> Trace {
    let (trace, _) = self.cpu.step(debug);
    trace
  }

  pub fn run(&mut self, debug: bool) -> () {
    loop {
      self.step(debug);
    }
  }
}
