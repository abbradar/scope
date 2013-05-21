#ifndef SCOPE_H_
#define SCOPE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/types.h>
#include <linux/ioctl.h>

#define SCOPE_VERSION "1.01"

enum scope_state {
  SCOPE_INIT = 0b1,
  SCOPE_READY = 0b10,
  SCOPE_TRIGGERING = 0b100, 
};

enum sweep_type {
  SWEEP_LINEAR = 0,
  SWEEP_LOG = ((__u64)1 << 33),
};

enum coupling_type {
  SCOPE_COUPLING_AC = 0,
  SCOPE_COUPLING_DC = 1,
  SCOPE_COUPLING_GND = 1 << 4,
};

enum trigger_dir {
  TRIGGER_UP = 0 << 2,
  TRIGGER_DOWN = 1 << 2,
};

enum power_led {
  POWER_LED_OFF = 0 << 4,
  POWER_LED_DIM = 1 << 4,
  POWER_LED_BRIGHT = 2 << 4,
};

#define SCOPE_MIN_OFFSET -120
#define SCOPE_MAX_OFFSET +127

struct channel_settings {
  unsigned volt_div;
  enum coupling_type coupling;
  __s8 y_pos;
};

struct scope_settings {
  struct channel_settings channel[2];
  __u8 trigg_level;
  unsigned long time_div;
  bool enable_trg;
  __u8 trg_ch;
  enum trigger_dir trg_dir;
  bool digi;
};

#define GENERATOR_MIN_OFFSET -127
#define GENERATOR_MAX_OFFSET +128

#define GENERATOR_MAX_AMPL 7

#define GENERATOR_MIN_CORRECTION -4
#define GENERATOR_MAX_CORRECTION 3

#define GENERATOR_MAX_FILTER 7

struct generator_settings {
  __s16 dc_offset;
  __u8 ampl;
  __s8 correction;
  enum power_led power_led;
  __u8 filter;
  bool enable;
};

struct freq_settings {
  __u32 start_freq; // hz
  __u32 end_freq;
  __u32 sweep_time; // in 100us units
  bool log_sweep;
};

#define SCOPE_TRANSIENT_TIME 0

#define SCOPE_DATA_SIZE 8192
#define SCOPE_TRANSIENT_SIZE 64
#define GENERATOR_WAVE_SIZE 512

#define IOCTL_MAGIC 0xBB // aBBradar :3
#define SIO_GET_VERSION _IOR(IOCTL_MAGIC, 1, const char)
#define SIO_SCOPE_SET _IOW(IOCTL_MAGIC, 2, struct scope_settings)
#define SIO_SCOPE_GET _IOR(IOCTL_MAGIC, 3, struct scope_settings)
#define SIO_GENERATOR_SET _IOW(IOCTL_MAGIC, 4, struct generator_settings)
#define SIO_GENERATOR_GET _IOR(IOCTL_MAGIC, 5, struct generator_settings)
#define SIO_FREQ_SET _IOW(IOCTL_MAGIC, 6, struct freq_settings)
#define SIO_FREQ_GET _IOR(IOCTL_MAGIC, 7, struct freq_settings)
#define SIO_WAVEFORM_SET _IOW(IOCTL_MAGIC, 8, const char)
#define SIO_STATUS _IOR(IOCTL_MAGIC, 9, enum scope_state)
#define SIO_TRIGGER _IO(IOCTL_MAGIC, 10)

#ifdef __cplusplus
}
#endif

#endif // SCOPE_H_
