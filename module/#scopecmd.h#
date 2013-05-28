#ifndef SCOPECMD_H_
#define SCOPECMD_H_

#include <linux/types.h>

#define SCOPE_VENDOR_ID 0x10CF
#define SCOPE_PRODUCT_ID 0x2501
#define SCOPE_TIMEOUT 1000
#define SCOPE_FW_TIMEOUT 10000

#define SCOPE_CMD_FW_START "\x08"
#define SCOPE_CMD_SHUTDOWN "\x13"

#define SCOPE_CMD_VERSION_GET "\x0F"
#define SCOPE_VERSION_SIZE 64
#define SCOPE_VERSION_DELIMITER '\r'

#define SCOPE_CMD_SET '\x0E'
#define SCOPE_CMD_SETTING_0 '\x80'
#define SCOPE_CMD_SETTING_1 7
#define GENERATOR_CMD_SETTING_0 '\x05'
#define GENERATOR_CMD_SETTING_1 4
#define GENERATOR_CMD_FREQ_SETTING_0 '\x02'
#define GENERATOR_CMD_FREQ_SETTING_1 '\x13'

#define SCOPE_SETTINGS_SIZE 10
#define GENERATOR_SETTINGS_SIZE 7
#define FREQ_SETTINGS_SIZE 22

#define SCOPE_TRANSIENT_DIV 0x02

struct uint32_char {
  __u32 k;
  char v;
};

extern const struct uint32_char kVoltDivTable[];
extern const struct uint32_char kTimeDivTable[];

#define SCOPE_CMD_RESET "\x09"
#define SCOPE_CMD_TRIGGER "\x0B"
#define SCOPE_TRIGGER_WAIT '\x4E'
#define SCOPE_TRIGGER_READY '\x44'
#define SCOPE_CMD_READ "\x0A"
#define SCOPE_CMD_TRANSIENT_READ "\x0C"

struct char_uint32 {
  char k;
  __u32 v;
};

extern const struct char_uint32 kClkTable[];

#define GENERATOR_CMD_WAVE_SETTING "\x04"

#define GENERATOR_CMD_START "\x06"

#endif // SCOPECMD_H_
