mod bus;
mod cartridge;
mod cpu;
mod mapper;
mod nes;
mod opcode;
mod util;

use crate::bus::Bus;
use crate::cartridge::Cartridge;
use crate::cpu::Cpu;
use crate::nes::Nes;

fn main() {
  let cartridge = Cartridge::load_rom("roms/tests/cpu/nestest/nestest.nes".to_string());
  let bus = Bus::new(cartridge);
  let cpu = Cpu::new(bus);
  let mut nes = Nes::new(cpu);

  nes.reset();
  nes.run();
}
