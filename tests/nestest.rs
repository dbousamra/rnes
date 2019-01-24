#[allow(dead_code)]
#[allow(unused_imports)]
use rnes::bus::Bus;
use rnes::nes::Nes;
use rnes::trace::Trace;

use std::fs::File;
use std::io;
use std::io::prelude::*;

#[test]
fn nestest() {
  let f = File::open("roms/tests/cpu/nestest/nestest.log").expect("Nestest log file not found");
  let f = io::BufReader::new(f);

  let mut nes = Nes::from_rom("roms/tests/cpu/nestest/nestest.nes".to_string());
  nes.reset();
  nes.cpu.pc = 0xC000;

  for line in f.lines() {
    let t1 = nes.step(false);
    let t2 = Trace::from_string(line.unwrap());
    assert_eq!(t1, t2);
  }
}
