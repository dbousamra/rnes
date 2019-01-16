pub fn make_word(lo: u8, hi: u8) -> u16 {
  (lo as u16) | (hi as u16) << 8
}

pub fn first_nibble(value: u16) -> u16 {
  let (lo, hi) = split_word(value);
  lo as u16
}

pub fn split_word(value: u16) -> (u8, u8) {
  ((value & 0xFF) as u8, (value >> 8) as u8)
}
