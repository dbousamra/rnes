extern crate rnes;

use rnes::nes::Nes;

fn main() {
  let mut nes = Nes::from_rom("roms/tests/cpu/nestest/nestest.nes".to_string());
  nes.reset();
  nes.run(true);
}
