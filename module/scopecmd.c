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
  {250, 0xC1},
  {625, 0xC2},
  {1250, 0xE0},
  {2500, 0xE1},
  {6250, 0xE2},
  {12500, 0xF0},
  {25000, 0xF1},
  {62500, 0xF2},
  {125000, 0xF8},
  {250000, 0xF9},
  {625000, 0xFA},
  {1250000, 0xFC},
  {2500000, 0xFD},
  {6250000, 0xFE},
  {12500000, 0x80},
  {25000000, 0x40},
  {0, 0}
};

const struct char_uint32 kClkTable[] = {
  {5, 6.25e6},
  {0, 12.5e6},
  {0, 0}
};

