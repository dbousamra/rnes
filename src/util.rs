

pub fn make_word(lo: u8, hi: u8) -> u16 {
  (lo as u16) | (hi as u16) << 8
}