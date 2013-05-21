#!/usr/bin/env python2

FW_BIN = '../res/fw.bin'
FW_TIMEOUT = 10000

ID_VENDOR = 0x10CF
ID_PRODUCT = 0x2501
FW_START = b'\x08'
VERSION_GET = b'\x0F'
VERSION_SIZE = 64
VERSION_DELIMITER = b'\r'
LOAD_SETTING = b'\x0E'
SCOPE_RESET = b'\x09'
SCOPE_TRIGGER = b'\x0B'
SCOPE_WAITING = b'\x4E'
SCOPE_READY = b'\x44'
SCOPE_READ = b'\x0A'
SCOPE_TRANSIENT = b'\x0C'
SCOPE_DATA_SIZE = 8192
SCOPE_TRANSIENT_SIZE = 64
GENERATOR_START = b'\x06'
GENERATOR_WAVE_SETTING = b'\x04'
GENERATOR_WAVE_SIZE = 512
SHUTDOWN_CMD = b'\x14'

def float_to_chr(float, max=0xff):
  if not 0 <= float <= 1:
    raise ValueError('Invalid float range')
  return chr(int(round(max * float)))

import struct

class ScopeSettings:
  SCOPE_SETTING = b'\x80'
  AVERAGE_OFFSET = float(0x78) / float(0xf7)

  VOLT_DIV_TABLE = { 10:   0x22,
                     30:   0x02,
                     100:  0x24,
                     300:  0x04,
                     1000: 0x28,
                     3000: 0x08,
                   }

  TIME_DIV_TABLE = { 5:      b'\x40',
                     10:     b'\x80',
                     20:     b'\xfe',
                     50:     b'\xfd',
                     100:    b'\xfc',
                     200:    b'\xfa',
                     500:    b'\xf9',
                     1000:   b'\xf8',
                     2000:   b'\xf2',
                     5000:   b'\xf1',
                     10000:  b'\xf0',
                     20000:  b'\xe2',
                     50000:  b'\xe1',
                     100000: b'\xe0',
                     200000: b'\xc2',
                     500000: b'\xc1',
                     0:      b'\x02',
                   }

  TRIGGER_UP = 0
  TRIGGER_DOWN = 1

  COUPLING_AC = 0
  COUPLING_DC = 1
  COUPLING_GND = 1 << 4

  def __init__(self):
    self.volt_div = [1000, 1000]
    self.couping = [self.COUPLING_DC, self.COUPLING_DC]
    self.y_position = [self.AVERAGE_OFFSET, self.AVERAGE_OFFSET]
    self.trigger_level = 0.5
    self.time_div = 1000
    self.trigger_ch = 0
    self.trigger = False
    self.trigger_edge = self.TRIGGER_UP
    self.digital = False

  def packet(self):
    packet = LOAD_SETTING + self.SCOPE_SETTING
    settings = ''
    for i in range(0, 2):
      settings += chr(self.VOLT_DIV_TABLE[self.volt_div[i]] \
          + self.couping[i])
    for i in range(0, 2):
      settings += float_to_chr(self.y_position[i], max=0xf7)
    settings += float_to_chr(self.trigger_level)
    settings += self.TIME_DIV_TABLE[self.time_div]
    if not self.trigger_edge in (self.TRIGGER_UP, self.TRIGGER_DOWN):
      raise ValueError("Invalid trigger edge value")
    settings += chr(self.trigger_ch + (self.trigger << 1) + (self.trigger_edge << 2) \
        + (self.digital << 3))
    packet += chr(len(settings)) + settings
    return packet

class GeneratorSettings:
  GENERATOR_SETTING = b'\x05'
  GENERATOR_FREQ_SETTING = b'\x02\x13'
  AVERAGE_DC_OFFSET = 0
  AVERAGE_AMPLITUDE = 7 / 2

  FILTER_TABLE = { 'SWEEP':  { 50000:   7,
                               150000:  6,
                               300000:  5,
                               500000:  4,
                               700000:  2,
                               1000000: 1,
                             },
                   'SINE':   { 50000:   7,
                               150000:  6,
                               300000:  5,
                               400000:  3,
                               500000:  2,
                               1000000: 1,
                             },
                   'SINE/X': { 5000:    7,
                               50000:   6,
                               500000:  1,
                             },
                   'SQUARE': { 1000000: 0,
                             },
                   'DC':     { 1000000: 7,
                             },
                   'OTHER':  { 50000:   7,
                               500000:  0,
                             },
                 }
  CLK_TABLE = { 5: 6.25 * 10**6, 
                0: 12.5 * 10**6,
              }
  PHASE_INCREMENT_SIZE = 2**44
  SWEEP_INCREMENT_SIZE = 2**64

  SWEEP_LINEAR = 0
  SWEEP_LOG = 2**33

  @staticmethod
  def _dc_offset_to_chr(dc):
    if not -5 <= dc <= 5:
      raise ValueError('Invalid DC offset')
    return chr(int(round((dc + 5.0) / 10 * 0xff)))

  @staticmethod
  def _correction_to_int(corr):
    if not -5 <= corr <= 5:
      raise ValueError("Invalid correction value")
    val = int(round(corr / 5.0 * 3.0))
    if corr >= 0:
      val += 4
    return val

  def __init__(self):
    self.dc_offset = self.AVERAGE_DC_OFFSET
    self.amplitude = self.AVERAGE_AMPLITUDE
    self.freq_range = 0
    self.relay_mode = 0
    self.correction = 0
    self.power_led = 2
    self.sweep = False
    self.sweep_type = self.SWEEP_LINEAR
    self.sweep_start = 0
    self.sweep_end = 0
    self.sweep_time = 10000
    self.frequency = 0
    self.type = 'OTHER'

  def _filter(self):
    return [w for (k, w) in sorted(self.FILTER_TABLE[self.type].items())
        if self.frequency < k][0]

  def packet(self):
    packet = LOAD_SETTING + self.GENERATOR_SETTING
    settings = ''
    settings += self._dc_offset_to_chr(self.dc_offset)
    if not 0 <= self.amplitude < 8:
      raise ValueError("Invalid amplitude")
    if not 0 <= self.freq_range < 8:
      raise ValueError("Invalid frequency range")
    if not 0 <= self.relay_mode < 4:
      raise ValueError("Invalid relay mode")
    settings += chr(self.amplitude + (self.freq_range << 3) + (self.relay_mode << 6))
    if not 0 <= self.power_led <= 2:
      raise ValueError("Invalid power LED level")
    settings += chr(self._correction_to_int(self.correction) \
        + (self.power_led << 4))
    settings += chr(self._filter() + (self.sweep << 3))
    packet += chr(len(settings)) + settings
    return packet

  def frequency_packet(self):
    filter = self._filter()
    clk = [w for (k, w) in sorted(self.CLK_TABLE.items()) if filter > k][-1]
    packet = LOAD_SETTING + self.GENERATOR_FREQ_SETTING
    sweep_increment = 2**59 * (self.sweep_end - self.sweep_start) / clk / self.sweep_time
    if filter > 5:
      sweep_increment *= 2
    if self.sweep_type == self.SWEEP_LINEAR:
      sweep_increment *= 2**5
    packet += struct.pack('<Q', round(sweep_increment))
    phase_increment = 2**44 * self.frequency / clk
    packet += struct.pack('<Q', round(phase_increment))[:6]
    sweep_complete = self.sweep_time / 10**2
    if filter > 5:
      sweep_complete /= 2
    if self.sweep_type == self.SWEEP_LOG:
      sweep_complete /= 8
      sweep_complete += self.sweep_type
    packet += struct.pack('<Q', round(sweep_complete))[:5]
    return packet

from itertools import *

def get_waveform(func, start, period):
  y = []
  for i in xrange(0, GENERATOR_WAVE_SIZE):
    y.append(func(start + float(i) / GENERATOR_WAVE_SIZE * period))
  miny = min(y)
  maxy = max(y)
  y = bytearray([int(round((i - miny) / (maxy - miny) * 0xff)) for i in y])
  return y

import usb.core

dev = usb.core.find(idVendor=ID_VENDOR, idProduct=ID_PRODUCT)
if dev is None:
  raise ValueError('Device not found')
dev.set_configuration()
cfg = dev.get_active_configuration()

interface_number = cfg[(0,0)].bInterfaceNumber
alternate_setting = usb.control.get_interface(dev, interface_number)
intf = usb.util.find_descriptor(
   cfg, bInterfaceNumber = interface_number,
    bAlternateSetting = alternate_setting
)

e_in = usb.util.find_descriptor(
    intf,
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_IN
)

e_out = usb.util.find_descriptor(
    intf,
    custom_match = \
    lambda e: \
        usb.util.endpoint_direction(e.bEndpointAddress) == \
        usb.util.ENDPOINT_OUT
)

assert e_in is not None and e_out is not None

e_out.write(FW_START)
fw = open(FW_BIN, 'r').read()
e_out.write(fw, timeout=FW_TIMEOUT)

e_out.write(VERSION_GET)
ver = e_in.read(VERSION_SIZE)
ver = ''.join(map(chr, ver[:ver.index(ord(VERSION_DELIMITER))]))
print(ver)

from math import *

scope_settings = ScopeSettings()
generator_settings = GeneratorSettings()
e_out.write(scope_settings.packet())
generator_settings.amplitude = 6
generator_settings.frequency = 500
generator_settings.type = 'SINE'
generator_settings.sweep = 1
e_out.write(generator_settings.packet())
e_out.write(generator_settings.frequency_packet())
wave = get_waveform(sin, 0, 2 * pi)
e_out.write(GENERATOR_WAVE_SETTING)
e_out.write(wave)
e_out.write(GENERATOR_START)

def wait_trigger():
  e_out.write(SCOPE_TRIGGER)
  b = None
  try:
    b = chr(e_in.read(1)[0])
  except usb.core.USBError:
    #e_in.read(63) # read junk data
    #shitty PyUSB
    e_out.write(SCOPE_TRIGGER)
  while b != SCOPE_READY:
    b = chr(e_in.read(1)[0])

def transient_init():
  wait_trigger()
  e_out.write(SCOPE_READ)
  e_in.read(SCOPE_DATA_SIZE)
  e_out.write(SCOPE_TRANSIENT)
  e_out.write(SCOPE_RESET)

e_out.write(SCOPE_RESET)
#transient_init()

from itertools import *
from collections import *

import matplotlib
matplotlib.use("Qt4Agg")
from matplotlib.animation import *
import matplotlib.pyplot as plt

fig = plt.figure()
lim = SCOPE_DATA_SIZE
ax = plt.axes(xlim=(0, lim), ylim=(-1.2, 1.2))
lines = [ax.plot([], [], color="g")[0], ax.plot([], [], color="r")[0]]
x = range(0, lim)
ys = [deque(maxlen=lim), deque(maxlen=lim)]

def oscilloscope(i):
  wait_trigger()
  e_out.write(SCOPE_READ)
  data = e_in.read(SCOPE_DATA_SIZE)
  for y, n in izip(ys, count()):
    y.extend(map(lambda x : (float(x) / 0x7f - 1) * scope_settings.volt_div[n]
        / 1000, data[n::2]))
  for l, y in izip(lines, ys):
    l.set_data(x[:len(y)], y)
  return lines

def transient(i):
  wait_trigger()
  e_out.write(SCOPE_TRANSIENT)
  data = e_in.read(SCOPE_TRANSIENT_SIZE)
  for y, n in izip(ys, count()):
    y.extend(map(lambda x : (float(x) / 0x7f - 1) * scope_settings.volt_div[n]
        / 1000, data[n::2]))
  for l, y in izip(lines, ys):
    l.set_data(x[:len(y)], y)
  return lines

def init():
  for i in lines:
    i.set_data([], [])
  return lines

anim = FuncAnimation(fig, oscilloscope, interval=0, init_func=init, blit=True)
plt.show()

try:
  e_out.write(SHUTDOWN_CMD)
except usb.core.USBError:
  pass
try:
  dev.reset()
except usb.core.USBError:
  pass
