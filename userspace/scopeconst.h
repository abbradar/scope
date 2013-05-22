#ifndef SCOPECONST_H
#define SCOPECONST_H

struct uint32_char {
  unsigned k;
  char v;
};

enum filter_type {
    SWEEP_WAVE,
    SINE_WAVE,
    SINE_DIVX_WAVE,
    SQUARE_WAVE,
    DC_WAVE,
    OTHER_WAVE,
};

struct filter_row {
  filter_type k;
  uint32_char *v;
};

extern const filter_row kFilterTable[];

#endif // SCOPECONST_H
