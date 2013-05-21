#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/kref.h>
#include <linux/fs.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/firmware.h>
#include <linux/kthread.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <asm/uaccess.h>
#include "scopecmd.h"
#include "scope.h"

#define SCOPE_PREFIX "pcsgu250: "

static struct usb_driver scope_driver;

struct scope_data {
  struct kref kref;

  struct usb_device *udev;
  struct usb_interface *interface;
  struct usb_anchor submitted;

  unsigned in_pipe;
  int in_interval;
  unsigned out_pipe;
  int out_interval;

  struct mutex io_mutex;

  struct urb *in_urb;
  unsigned char *in_buffer;
  size_t in_pos, in_size;
  void (*in_callback)(struct scope_data *);
  struct completion in_completion;
  bool in_busy;
  int in_avaliable;
  int in_errors;
  spinlock_t in_err_lock;

  spinlock_t state_lock;
  enum scope_state state;
  struct task_struct *init_thread;
  char version[SCOPE_VERSION_SIZE];

  spinlock_t settings_lock;
  struct scope_settings scope_settings;
  struct generator_settings generator_settings;
  struct freq_settings freq_settings;
};

static void scope_delete(struct kref *kref) {
  struct scope_data *dev = container_of(kref, struct scope_data, kref);
  usb_free_coherent(dev->in_urb->dev, SCOPE_DATA_SIZE, dev->in_buffer,
      dev->in_urb->transfer_dma);
  usb_free_urb(dev->in_urb);
  if (dev->init_thread) kthread_stop(dev->init_thread);
  usb_put_dev(dev->udev);
  kfree(dev);
}

static int scope_open(struct inode *inode, struct file *file) {
  int subminor = iminor(inode);
  struct usb_interface *interface = usb_find_interface(&scope_driver, subminor);
  if (!interface) {
    pr_err(SCOPE_PREFIX "error, can't find device for minor %d\n", subminor);
    return -ENODEV;
  }
  struct scope_data *dev = usb_get_intfdata(interface);
  if (!dev) {
    return -ENODEV;
  }
  int rv = usb_autopm_get_interface(interface);
  if (rv) return rv;
  kref_get(&dev->kref);
  file->private_data = dev;
  return nonseekable_open(inode, file);
}

static int scope_release(struct inode *inode, struct file *file) {
  struct scope_data *dev = file->private_data;
  if (!dev) return -ENODEV;

  mutex_lock(&dev->io_mutex);
  if (dev->interface)
    usb_autopm_put_interface(dev->interface);
  mutex_unlock(&dev->io_mutex);

  kref_put(&dev->kref, scope_delete);
  return 0;
}

struct usb_send {
  int errors;
  struct completion completion;
};

static void scope_write_callback(struct urb *urb) {
  struct usb_send *send = urb->context;
  if (urb->status) {
    if (!(urb->status == -ENOENT ||
          urb->status == -ECONNRESET ||
          urb->status == -ESHUTDOWN)) {
      send->errors = urb->status;
    }
  }
  complete_all(&send->completion);
}

static int scope_send(struct scope_data *dev, const char *cmd,
    const size_t count) {
  if (!count) return 0;
  
  struct urb *urb = usb_alloc_urb(0, GFP_KERNEL);
  if (!urb) return -ENOMEM;
  char *buff = usb_alloc_coherent(dev->udev, count, GFP_KERNEL, &urb->transfer_dma);
  if (!buff) return -ENOMEM;
  memcpy(buff, cmd, count);
  struct usb_send send;
  send.errors = 0;
  init_completion(&send.completion);
  mutex_lock(&dev->io_mutex);
  if (!dev->interface) {
    mutex_unlock(&dev->io_mutex);
    return -ENODEV;
  }
  usb_fill_int_urb(urb, dev->udev, dev->out_pipe, buff, count,
      &scope_write_callback, &send, dev->out_interval);
  urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
  usb_anchor_urb(urb, &dev->submitted);
  int rv = usb_submit_urb(urb, GFP_KERNEL);
  mutex_unlock(&dev->io_mutex);
  if (rv) {
    dev_err(&dev->udev->dev, "failed to submit write urb, error %d\n", rv);
    usb_unanchor_urb(urb);
    usb_free_coherent(dev->udev, count, buff, urb->transfer_dma);
    usb_free_urb(urb);
    return rv;
  }
  wait_for_completion(&send.completion);
  usb_free_coherent(dev->udev, count, buff, urb->transfer_dma);
  rv = (send.errors == -EPIPE) ? -EIO : send.errors;
  return rv;
}

static void scope_read_callback(struct urb *urb) {
  struct scope_data *dev = (struct scope_data *)urb->context;
  if (urb->status) {
    if (!(urb->status == -ENOENT ||
          urb->status == -ECONNRESET ||
          urb->status == -ESHUTDOWN)) {
      dev_err(&dev->udev->dev, "nonzero read status received: %d\n",
          urb->status);
      spin_lock(&dev->in_err_lock);
      dev->in_errors = urb->status;
      spin_unlock(&dev->in_err_lock);
    }
  }
  if (dev->in_callback) dev->in_callback(dev);
  spin_lock(&dev->in_err_lock);
  dev->in_busy = false;
  spin_unlock(&dev->in_err_lock);
  complete_all(&dev->in_completion);
}

static int scope_receive(struct scope_data *dev, size_t count, bool block,
    void (*callback)(struct scope_data *)) {
  if (count == 0) return 0;

  int rv = mutex_lock_interruptible(&dev->io_mutex);
  if (rv) return rv;
  if (!dev->interface) {
    mutex_unlock(&dev->io_mutex);
    return -ENODEV;
  }
  unsigned long flags;
  spin_lock_irqsave(&dev->in_err_lock, flags);
  rv = dev->in_errors;
  if (rv) {
    dev->in_errors = 0;
  }
  bool in_busy = dev->in_busy;
  spin_unlock_irqrestore(&dev->in_err_lock, flags);
  rv = (rv == -EPIPE) ? -EIO : rv;
  if (rv) goto error;
  if (in_busy) {
    wait_for_completion(&dev->in_completion);
    INIT_COMPLETION(dev->in_completion);
  }

  spin_lock_irqsave(&dev->in_err_lock, flags);
  in_busy = true;
  spin_unlock_irqrestore(&dev->in_err_lock, flags);

  dev->in_callback = callback;
  usb_fill_int_urb(dev->in_urb, dev->udev, dev->in_pipe,
      dev->in_buffer, count, &scope_read_callback, dev, dev->in_interval);
  dev->in_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
  usb_anchor_urb(dev->in_urb, &dev->submitted);
  
  rv = usb_submit_urb(dev->in_urb, GFP_KERNEL);
  if (rv) {
    spin_lock_irqsave(&dev->in_err_lock, flags);
    dev->in_busy = false;
    spin_unlock_irqrestore(&dev->in_err_lock, flags);
    complete_all(&dev->in_completion);
  }
  if (block) {
    wait_for_completion(&dev->in_completion);
    rv = dev->in_errors;
    mutex_unlock(&dev->io_mutex);
    rv = (rv == -EPIPE) ? -EIO : rv;
  } else {
    mutex_unlock(&dev->io_mutex);
  }
  return rv;

error:
  mutex_unlock(&dev->io_mutex);
  return rv;
}

static const struct scope_settings default_scope_settings = {
  .channel = {
    { .volt_div = 1000,
      .coupling = SCOPE_COUPLING_DC,
      .y_pos = 0,
    },
    { .volt_div = 1000,
      .coupling = SCOPE_COUPLING_DC,
      .y_pos = 0,
    },
  },
  .trigg_level = 0x7f,
  .time_div = 1000,
  .enable_trg = false,
  .trg_ch = 0,
  .trg_dir = TRIGGER_UP,
  .digi = false,
};

static const struct generator_settings default_generator_settings = {
  .dc_offset = 0,
  .ampl = 4,
  .correction = 0,
  .power_led = POWER_LED_BRIGHT,
  .filter = 7,
  .enable = false,
};

static const struct freq_settings default_freq_settings = {
  .start_freq = 0,
  .end_freq = 0,
  .sweep_time = 1,
  .log_sweep = false,
};

#define SCOPE_SETTINGS_SIZE 10

static int scope_fill_scope_settings(const struct scope_settings *settings,
    unsigned char *buf) {
  buf[0] = SCOPE_CMD_SET;
  buf[1] = SCOPE_CMD_SETTING_0;
  buf[2] = SCOPE_CMD_SETTING_1;
  for (int i = 0; i < 2; ++i) {
    const struct channel_settings *channel = &settings->channel[i];
    if (channel->y_pos < SCOPE_MIN_OFFSET ||
        channel->y_pos > SCOPE_MAX_OFFSET) return 1;
    if (channel->coupling != SCOPE_COUPLING_DC &&
        channel->coupling != SCOPE_COUPLING_AC &&
        channel->coupling != SCOPE_COUPLING_GND)
      return -EINVAL;
    const struct uint32_char *volt_div = kVoltDivTable;
    while (volt_div->k && volt_div->k != channel->volt_div) ++volt_div;
    if (!volt_div->k) return -EINVAL;
    buf[3 + i] = volt_div->v | channel->coupling;
    buf[5 + i] = (int)channel->y_pos - SCOPE_MIN_OFFSET;
  }
  buf[7] = settings->trigg_level;
  if (settings->time_div == SCOPE_TRANSIENT_TIME) {
    if (settings->enable_trg) return -EINVAL;
    buf[8] = SCOPE_TRANSIENT_DIV;
  } else {
    const struct uint32_char *time_div = kTimeDivTable;
    while (time_div->k && time_div->k != settings->time_div) ++time_div;
    if (!time_div->k) return -EINVAL;
    buf[8] = time_div->v;
  }
  if (settings->trg_ch >= 2) return -EINVAL;
  if (settings->trg_dir != TRIGGER_UP &&
      settings->trg_dir != TRIGGER_DOWN) return -EINVAL;
  buf[9] = settings->trg_ch | ((settings->enable_trg ? 1 : 0) << 1) |
      settings->trg_dir | ((settings->digi ? 1 : 0) << 3);
  return 0;
}

#define GENERATOR_SETTINGS_SIZE 7

static int scope_fill_generator_settings(const struct generator_settings *settings,
    unsigned char *buf) {
  buf[0] = SCOPE_CMD_SET;
  buf[1] = GENERATOR_CMD_SETTING_0;
  buf[2] = GENERATOR_CMD_SETTING_1;
  if (settings->dc_offset < GENERATOR_MIN_OFFSET ||
      settings->dc_offset > GENERATOR_MAX_OFFSET) return -EINVAL;
  buf[3] = settings->dc_offset - GENERATOR_MIN_OFFSET;
  if (settings->ampl > GENERATOR_MAX_AMPL) return -EINVAL;
  buf[4] = settings->ampl;
  if (settings->correction < GENERATOR_MIN_CORRECTION ||
      settings->correction > GENERATOR_MAX_CORRECTION) return -EINVAL;
  if (settings->power_led != POWER_LED_OFF &&
      settings->power_led != POWER_LED_DIM &&
      settings->power_led != POWER_LED_BRIGHT) return -EINVAL;
  buf[5] = (settings->correction - GENERATOR_MIN_CORRECTION) | settings->power_led;
  if (settings->filter > GENERATOR_MAX_FILTER) return -EINVAL;
  buf[6] = settings->filter | ((settings->enable ? 1 : 0) << 3);
  return 0;
}

#define high_bit(x) ({ \
  int pos = 0; \
  for (int i = 0; i < sizeof(x) * 8; ++i) { \
    if ((x >> i) & 1) pos = i + 1; \
  } \
  pos; \
})
#define FREQ_SETTINGS_SIZE 22

static int scope_fill_freq_settings(const struct freq_settings *freq,
    const struct generator_settings *generator, unsigned char *buf) {
  buf[0] = SCOPE_CMD_SET;
  buf[1] = GENERATOR_CMD_FREQ_SETTING_0;
  buf[2] = GENERATOR_CMD_FREQ_SETTING_1;
  __u32 clk = (generator->filter <= 5) ? 12500000 : 6250000;
  if (!freq->sweep_time) return -EINVAL;
  if (freq->start_freq > clk ||
      freq->end_freq > clk) return -EINVAL;
  if (freq->start_freq > freq->end_freq) return -EINVAL;
  // here go machine arithmetic checks and optimizations
  int clk_shift = 0;
  while (!(clk & 1)) {
    ++clk_shift;
    clk >>= 1;
  }
  // for sweep_inc, overflow is possible; check for it
  __u32 freq_diff = freq->end_freq - freq->start_freq;
  int bits_diff = high_bit(freq_diff);
  // determine sum of bits to shift
  int bits_shift = 59 -clk_shift + (generator->filter > 5 ? 1 : 0) +
      (freq->log_sweep ? 0 : 5);
  if(bits_diff + bits_shift - high_bit(clk) - high_bit(freq->sweep_time) > 64)
      return -EOVERFLOW;
  // determine first shift
  __u32 sweep_time_min = freq->sweep_time;
  // we can try to minify sweep time a bit
  while (!(sweep_time_min & 1)) {
    --bits_shift;
    sweep_time_min >>= 1;
  }
  int curr_shift = min(bits_shift + bits_diff, 64) - bits_diff;
  // first, shift to maximum possible value and multiply
  // then, divide by multiplication of two dividers and do remained shifting
  __u64 sweep_inc = ((((__u64)1 << curr_shift) * freq_diff) /
      ((__u64)clk * (__u64)sweep_time_min)) << (bits_shift - curr_shift);
  __le64 sweep_inc_le = cpu_to_le64(sweep_inc);
  memcpy(buf + 3, &sweep_inc_le, 8);
  __u64 phase_inc = (((__u64)1 << (44 - clk_shift)) * freq->start_freq) / clk;
  if (phase_inc >= (__u64)1 << 49) return -EOVERFLOW;
  __le64 phase_inc_le = cpu_to_le64(phase_inc);
  memcpy(buf + 11, &phase_inc_le, 6);
  __u32 sweep_compl = freq->sweep_time << (generator->filter > 5 ? 1 : 0) <<
      (freq->log_sweep ? 3 : 0);
  __le32 sweep_compl_le = cpu_to_le32(sweep_compl);
  memcpy(buf + 17, &sweep_compl_le, 4);
  buf[21] = freq->log_sweep ? 2 : 0;
  return 0;
}

static ssize_t scope_read(struct file *filp, char __user *buf, size_t count,
    loff_t *off) {
  return 0;
}

#define goto_on_error(cmd, dev, label, err...) \
  if (cmd) { \
    dev_err(dev, err); \
    goto label; \
  }

#define free_on_error(cmd, dev, label, freecmd, err...) \
  if (cmd) { \
    dev_err(dev, err); \
    freecmd; \
    goto label; \
  }

static long scope_io_get_version(struct scope_data *dev, char __user *buf) {
  unsigned long flags;
  spin_lock_irqsave(&dev->state_lock, flags);
  enum scope_state state = dev->state;
  spin_unlock_irqrestore(&dev->state_lock, flags);
  if (!(state & SCOPE_INIT)) return -EBUSY;
  copy_to_user(buf, dev->version, strlen(dev->version));
  return 0;
}

static int scope_wait_trigger(struct scope_data *dev) {
  int rv = scope_send(dev, SCOPE_CMD_TRIGGER, sizeof(SCOPE_CMD_TRIGGER) - 1);
  if (rv) return rv;
  dev->in_buffer[0] = '\0';
  while (dev->in_buffer[0] != SCOPE_TRIGGER_READY) {
    rv = scope_receive(dev, 1, true, NULL);
    if (rv) return rv;
  }
  return 0;
}

static long scope_io_scope_set(struct scope_data *dev,
    struct scope_settings __user *settings) {
  unsigned long flags;
  spin_lock_irqsave(&dev->state_lock, flags);
  enum scope_state state = dev->state;
  dev->state &= ~SCOPE_READY;
  spin_unlock_irqrestore(&dev->state_lock, flags);
  if (!(state & SCOPE_READY)) return -EBUSY;
  struct scope_settings sc_set;
  copy_from_user(&sc_set, settings, sizeof(sc_set));
  char buf[SCOPE_SETTINGS_SIZE];
  int rv = scope_fill_scope_settings(&sc_set, buf);
  if (rv) return rv;
  rv = scope_send(dev, buf, SCOPE_SETTINGS_SIZE);
  if (rv) return rv;
  spin_lock_irqsave(&dev->settings_lock, flags);
  memcpy(&dev->scope_settings, &sc_set, sizeof(sc_set));
  spin_unlock_irqrestore(&dev->settings_lock, flags);
  rv = scope_send(dev, SCOPE_CMD_RESET, sizeof(SCOPE_CMD_RESET) - 1);
  goto_on_error(rv, &dev->udev->dev, exit, "error resetting scope\n");
  if (sc_set.time_div == SCOPE_TRANSIENT_TIME) {
  }
exit:
  spin_lock_irqsave(&dev->state_lock, flags);
  dev->state |= ~SCOPE_READY;
  spin_unlock_irqrestore(&dev->state_lock, flags);
  return rv;
}

static long scope_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
  struct scope_data *dev = filp->private_data;
  switch (cmd) {
    case SIO_GET_VERSION:
      return scope_io_get_version(dev, (char __user *)arg);
    default: return -ENOTTY;
  }
}

static const struct file_operations scope_ops = {
  .owner = THIS_MODULE,
  .open = scope_open,
  .read = scope_read,
  .unlocked_ioctl = scope_ioctl,
  .llseek = no_llseek,
  .release = scope_release,
};

static struct usb_class_driver scope_class = {
  .name = "scope%d",
  .fops = &scope_ops,
};

static void scope_disconnect(struct usb_interface *interface) {
  struct scope_data *dev = usb_get_intfdata(interface);

  int minor = interface->minor;
  dev_info(&interface->dev, "disconnecting scope%d\n", minor);
  usb_deregister_dev(interface, &scope_class);
  
  mutex_lock(&dev->io_mutex);
  dev->interface = NULL;
  mutex_unlock(&dev->io_mutex);

  usb_kill_anchored_urbs(&dev->submitted);
  kref_put(&dev->kref, scope_delete);
  
  dev_info(&interface->dev, "scope%d disconnected\n", minor);
}

static int scope_device_init(void *data) {
  struct scope_data *dev = (struct scope_data *)data;

  const struct firmware *fw;
  int rv = request_firmware(&fw, "pcsgu250.bin", &dev->interface->dev);
  goto_on_error(rv, &dev->udev->dev, exit, "could not load firmware\n");
  rv = scope_send(dev, SCOPE_CMD_FW_START, sizeof(SCOPE_CMD_FW_START) - 1);
  free_on_error(rv, &dev->udev->dev, exit, release_firmware(fw),
      "failed firmware load request\n");
  dev_notice(&dev->udev->dev, "loading firmware\n");
  rv = scope_send(dev, fw->data, fw->size);
  release_firmware(fw);
  goto_on_error(rv, &dev->udev->dev, exit, "failed firmware load\n");
  rv = scope_send(dev, SCOPE_CMD_VERSION_GET, sizeof(SCOPE_CMD_VERSION_GET) - 1);
  goto_on_error(rv, &dev->udev->dev, exit, "failed version request\n");
  rv = scope_receive(dev, SCOPE_VERSION_SIZE - 1, true, NULL);
  rv = scope_receive(dev, SCOPE_VERSION_SIZE, true, NULL);
  goto_on_error(rv, &dev->udev->dev, exit, "failed getting version: %i\n", rv);
  unsigned char *ver = (unsigned char *)memchr(dev->in_buffer,
      SCOPE_VERSION_DELIMITER, SCOPE_VERSION_SIZE);
  if (!ver) {
    dev_warn(&dev->udev->dev, "invalid version string!\n");
    memcpy(dev->version, "invalid", sizeof("invalid") + 1);
  } else {
    size_t pos = ver - dev->in_buffer;
    memcpy(dev->version, dev->in_buffer, pos);
    dev->version[pos] = '\0';
    if (strcmp(dev->version, SCOPE_VERSION)) {
      dev_warn(&dev->udev->dev, "possibly incompatible firmware version\n");
    }
  }
  dev_info(&dev->udev->dev, "firmware loaded, version %s\n", dev->version);
  char *buff = kmalloc(GENERATOR_WAVE_SIZE, GFP_KERNEL);
  goto_on_error(!buff, &dev->udev->dev, exit, "out of memory\n");
  rv = scope_fill_scope_settings(&dev->scope_settings, buff);
  free_on_error(rv, &dev->udev->dev, exit, kfree(buff),
      "scope settings filling error!\n");
  rv = scope_send(dev, buff, SCOPE_SETTINGS_SIZE);
  free_on_error(rv, &dev->udev->dev, exit, kfree(buff),
      "failed initial scope settings set\n");
  rv = scope_fill_generator_settings(&dev->generator_settings, buff);
  free_on_error(rv, &dev->udev->dev, exit, kfree(buff),
      "generator settings filling error!\n");
  rv = scope_send(dev, buff, GENERATOR_SETTINGS_SIZE);
  free_on_error(rv, &dev->udev->dev, exit, kfree(buff),
      "failed initial generator settings set\n");
  rv = scope_fill_freq_settings(&dev->freq_settings, &dev->generator_settings, buff);
  free_on_error(rv, &dev->udev->dev, exit, kfree(buff),
      "scope freq filling error!\n");
  rv = scope_send(dev, buff, FREQ_SETTINGS_SIZE);
  free_on_error(rv, &dev->udev->dev, exit, kfree(buff),
      "failed initial freq settings set\n");
  rv = scope_send(dev, GENERATOR_CMD_WAVE_SETTING,
      sizeof(GENERATOR_CMD_WAVE_SETTING) - 1);
  free_on_error(rv, &dev->udev->dev, exit, kfree(buff),
      "failed initial waveform set request\n");
  memset(buff, 0x7f, GENERATOR_WAVE_SIZE);
  rv = scope_send(dev, buff, GENERATOR_WAVE_SIZE);
  free_on_error(rv, &dev->udev->dev, exit, kfree(buff),
      "failed initial waveform set\n");
  kfree(buff);
  rv = scope_send(dev, GENERATOR_CMD_START, sizeof(GENERATOR_CMD_START) - 1);
  goto_on_error(rv, &dev->udev->dev, exit, "failed initial generator start\n");
  rv = scope_send(dev, SCOPE_CMD_RESET, sizeof(SCOPE_CMD_RESET) - 1);
  goto_on_error(rv, &dev->udev->dev, exit, "failed initial scope reset\n");

  dev_info(&dev->udev->dev, "successful initialization\n");
  unsigned long flags;
  spin_lock_irqsave(&dev->state_lock, flags);
  dev->state = SCOPE_INIT | SCOPE_READY;
  spin_unlock_irqrestore(&dev->state_lock, flags);
  
exit:
  kref_put(&dev->kref, scope_delete);
  set_current_state(TASK_INTERRUPTIBLE);
  while (!kthread_should_stop()) {
    schedule();
    set_current_state(TASK_INTERRUPTIBLE);
  }
  return 0;
}

static int scope_probe(struct usb_interface *interface,
    const struct usb_device_id *id) {
  int rv = -ENOMEM;
  struct scope_data *dev = kzalloc(sizeof(*dev), GFP_KERNEL);
  if (!dev) {
    dev_err(&interface->dev, "out of memory\n");
    goto error;
  }
  dev->udev = usb_get_dev(interface_to_usbdev(interface));

  struct usb_host_interface *iface_desc = interface->cur_altsetting;
  for (int i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
    struct usb_endpoint_descriptor *endpoint = &iface_desc->endpoint[i].desc;
    if (!dev->in_pipe && usb_endpoint_is_int_in(endpoint)) {
      /* we found an interrupt in endpoint */
      dev->in_pipe = usb_rcvintpipe(dev->udev, endpoint->bEndpointAddress);
      dev->in_interval = endpoint->bInterval;
      dev->in_urb = usb_alloc_urb(0, GFP_KERNEL);
      goto_on_error(!dev->in_urb, &interface->dev, error,
          "could not allocate in_urb\n");
      dev->in_buffer = usb_alloc_coherent(dev->udev, SCOPE_DATA_SIZE, GFP_KERNEL,
          &dev->in_urb->transfer_dma);
      goto_on_error(!dev->in_buffer, &interface->dev, error,
          "could not allocate in_buffer\n");
      init_completion(&dev->in_completion);
    } else if (!dev->out_pipe && usb_endpoint_is_int_out(endpoint)) {
      /* we found an interrupt out endpoint */
      dev->out_pipe = usb_sndintpipe(dev->udev, endpoint->bEndpointAddress);
      dev->out_interval = endpoint->bInterval;
    }
  }
  goto_on_error(!dev->in_urb, &interface->dev, error,
      "could not find both interrupt-in and interrupt-out endpoints\n");

  kref_init(&dev->kref);
  kref_get(&dev->kref);
  dev->interface = interface;
  init_usb_anchor(&dev->submitted);
  spin_lock_init(&dev->in_err_lock);
  mutex_init(&dev->io_mutex);
  spin_lock_init(&dev->state_lock);
  spin_lock_init(&dev->settings_lock);
  memcpy(&dev->scope_settings, &default_scope_settings,
      sizeof(default_scope_settings));
  memcpy(&dev->generator_settings, &default_generator_settings,
      sizeof(default_generator_settings));
  memcpy(&dev->freq_settings, &default_freq_settings,
      sizeof(default_freq_settings));

  usb_set_intfdata(interface, dev);
  rv = usb_register_dev(interface, &scope_class);
  free_on_error(rv, &interface->dev, error, usb_set_intfdata(interface, NULL),
      "could not get minor\n");
  dev->init_thread = kthread_run(&scope_device_init, (void *)dev, "pcsgu250_init");
  dev_info(&interface->dev, "device now attached to scope%d\n",
      interface->minor);
  return 0;

error:
  if (dev)
    /* this frees allocated memory */
    kref_put(&dev->kref, scope_delete);
  return rv;
}

static const struct usb_device_id scope_id_table[] = {
	{ USB_DEVICE(SCOPE_VENDOR_ID, SCOPE_PRODUCT_ID) },
	{ }
};

static int scope_pre_reset(struct usb_interface *intf) {
  return 0;
}

static int scope_post_reset(struct usb_interface *intf) {
  return 0;
}

MODULE_DEVICE_TABLE(usb, scope_id_table);
static struct usb_driver scope_driver = {
  .name = "pcsgu250",
  .probe = scope_probe,
  .disconnect = scope_disconnect,
  .id_table = scope_id_table,
  .pre_reset = scope_pre_reset,
  .post_reset = scope_post_reset,
};

module_usb_driver(scope_driver);

MODULE_AUTHOR("Nikolay Amiantov");
MODULE_DESCRIPTION("Velleman PCSGU250 oscilloscope and generator USB driver");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");
