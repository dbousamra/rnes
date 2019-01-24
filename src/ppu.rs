pub struct Ppu {
  register: u8,
  control: Control,
  status: Status,
}

bitfield!{
  #[derive(Copy, Clone)]
  pub struct Status(u8);

  impl Debug;

  pub sprite_overflow, set_sprite_overflow: 5;
  pub sprite_zero_hit, set_sprite_zero_hit: 6;
  pub vblank,          set_vblank:          7;
  pub get,             _:                   7, 0;
}

bitfield!{
    #[derive(Copy, Clone)]
    pub struct Control(u8);

    impl Debug;

    pub nametable,          _: 1, 0;
    pub vertical_increment, _:    2;
    pub sprite_table,       _:    3;
    pub background_table,   _:    4;
    pub large_sprites,      _:    5;
    pub slave,              _:    6;
    pub nmi_on_vblank,      _:    7;
}

impl Ppu {
  pub fn new() -> Ppu {
    Ppu {
      register: 0,
      control: Control(0),
      status: Status(0),
    }
  }

  pub fn reset(&mut self) -> () {
    self.register = 0;
    self.control = Control(0);
    self.status = Status(0);
  }

  pub fn read_register(&mut self, address: u16) -> u8 {
    let result = match address {
      0x2002 => self.read_status(),
      0x2004 => self.read_oam_data(address),
      0x2007 => self.read_data(address),
      _ => self.register,
    };

    self.register = result;

    result
  }

  fn read_status(&mut self) -> u8 {
    let result = self.status.get();
    self.status.set_vblank(false);
    // self.latch = false;
    // self.vblank_suppress = true;
    result | (self.register & 0b11111)
  }

  fn read_oam_data(&mut self, address: u16) -> u8 {
    unimplemented!()
  }

  fn read_data(&mut self, address: u16) -> u8 {
    unimplemented!()
  }

  pub fn write_register(&mut self, address: u16, value: u8) -> () {
    self.register = value;
    match address {
      0x2000 => self.write_control(value),
      _ => panic!("Erroneous PPU register write at {:X}", address),
    }
  }

  fn write_control(&mut self, value: u8) -> () {
    self.control = Control(value)
  }
}
