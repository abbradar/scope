#include "scopecmd.h"

const struct uint32_char kVoltDivTable[] = {
  {10, 0x22},
  {30, 0x02},
  {100, 0x24},
  {300, 0x04},
  {1000, 0x28},
  {3000, 0x08},
  {0, 0}
};

const struct uint32_char kTimeDivTable[] = {
  {5, 0x40},
  {10, 0x80},
  {20, 0xFE},
  {50, 0xFD},
  {100, 0xFC},
  {200, 0xFA},
  {500, 0xF9},
  {1000, 0xF8},
  {2000, 0xF2},
  {5000, 0xF1},
  {10000, 0xF0},
  {20000, 0xE2},
  {50000, 0xE1},
  {100000, 0xE0},
  {200000, 0xC2},
  {500000, 0xC1},
  {0, 0}
};

const struct char_uint32 kClkTable[] = {
  {5, 6.25e6},
  {0, 12.5e6},
  {0, 0}
};

