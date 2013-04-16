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
GENERATOR_WAVE_SETTING = b'\x04'
GENERATOR_FREQ_SETTING = b'\x02\x13'
GENERATOR_WAVE_SIZE = 512
GENERATOR_START = b'\x06'
SHUTDOWN_CMD = b'\x14'

def float_to_chr(float, max=0xff):
  if not 0 <= float <= 1:
    raise ValueError('Invalid float range')
  return chr(int(round(max * float)))

class ScopeSettings:
  SCOPE_SETTING = chr(0x80)
  AVERAGE_OFFSET = float(0x78) / float(0xf7)

  VOLT_DIV_TABLE = { 10:   0x22,
                     30:   0x02,
                     100:  0x24,
                     300:  0x04,
                     1000: 0x28,
                     3000: 0x08,
                   }

  TIME_DIV_TABLE = { 5:      chr(0x40),
                     10:     chr(0x80),
                     20:     chr(0xfe),
                     50:     chr(0xfd),
                     100:    chr(0xfc),
                     200:    chr(0xfa),
                     500:    chr(0xf9),
                     1000:   chr(0xf8),
                     2000:   chr(0xf2),
                     5000:   chr(0xf1),
                     10000:  chr(0xf0),
                     20000:  chr(0xe2),
                     50000:  chr(0xe1),
                     100000: chr(0xe0),
                     200000: chr(0xc2),
                     500000: chr(0xc1),
                     0:      chr(0x02),
                   }

  TRIGGER_UP = 0
  TRIGGER_DOWN = 1

  COUPLING_AC = 0
  COUPLING_DC = 1
  COUPLING_GND = 1 << 4

  def __init__(self):
    self.volt_div = [1000, 1000]
    self.couping = [ScopeSettings.COUPLING_DC, ScopeSettings.COUPLING_DC]
    self.y_position = [ScopeSettings.AVERAGE_OFFSET, ScopeSettings.AVERAGE_OFFSET]
    self.trigger_level = 0
    self.time_div = 1000
    self.trigger_ch = 0
    self.trigger = False
    self.trigger_edge = ScopeSettings.TRIGGER_UP
    self.digital = False

  def packet(self):
    packet = LOAD_SETTING + ScopeSettings.SCOPE_SETTING
    settings = ''
    for i in range(0, 2):
      settings += chr(ScopeSettings.VOLT_DIV_TABLE[self.volt_div[i]] \
          + self.couping[i])
    for i in range(0, 2):
      settings += float_to_chr(self.y_position[i], max=0xf7)
    settings += float_to_chr(self.trigger_level)
    settings += ScopeSettings.TIME_DIV_TABLE[self.time_div]
    if not self.trigger_edge in (ScopeSettings.TRIGGER_UP, ScopeSettings.TRIGGER_DOWN):
      raise ValueError("Invalid trigger edge value")
    settings += chr(self.trigger_ch + (self.trigger << 1) + (self.trigger_edge << 2) \
        + (self.digital << 3))
    packet += chr(len(settings)) + settings
    return packet

class GeneratorSettings:
  GENERATOR_SETTING = chr(0x05)
  AVERAGE_AMPLITUDE = 8 / 2

  @staticmethod
  def _dc_offset_to_chr(dc):
    if not -5 <= dc <= 5:
      raise ValueError('Invalid DC offset')
    return chr(int(round((dc + 5) / 10 * 0xff)))

  @staticmethod
  def _correction_to_int(corr):
    if not -5 <= corr <= 5:
      raise ValueError("Invalid correction value")
    val = int(round(corr / 5 * 3))
    if corr >= 0:
      val += 4
    return val

  def __init__(self):
    self.dc_offset = 0
    self.amplitude = GeneratorSettings.AVERAGE_AMPLITUDE
    self.freq_range = 0
    self.relay_mode = 0
    self.correction = 0
    self.power_led = 2
    self.filter = 1
    self.sweep = False

  def packet(self):
    packet = LOAD_SETTING + GeneratorSettings.GENERATOR_SETTING
    settings = ''
    settings += GeneratorSettings._dc_offset_to_chr(self.dc_offset)
    if not 0 <= self.amplitude < 8:
      raise ValueError("Invalid amplitude")
    if not 0 <= self.freq_range < 8:
      raise ValueError("Invalid frequency range")
    if not 0 <= self.relay_mode < 4:
      raise ValueError("Invalid relay mode")
    settings += chr(self.amplitude + (self.freq_range << 3) + (self.relay_mode << 6))
    if not 0 <= self.power_led <= 2:
      raise ValueError("Invalid power LED level")
    settings += chr(GeneratorSettings._correction_to_int(self.correction) \
        + (self.power_led << 4))
    if not 0 <= self.filter < 8:
      raise ValueError("Invalid filter setting")
    settings += chr(self.filter + (self.sweep << 3))
    packet += chr(len(settings)) + settings
    return packet

def get_waveform(func, start, period):
  result = []
  for i in irange(0, GENERATOR_WAVE_SIZE):
    result.append(func(start + i / GENERATOR_WAVE_SIZE * period))
  return result

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

scope_settings = ScopeSettings()
generator_settings = GeneratorSettings()
e_out.write(scope_settings.packet())
e_out.write(generator_settings.packet())

def transient_init():
  wait_trigger()
  e_out.write(SCOPE_READ)
  e_in.read(SCOPE_DATA_SIZE)
  e_in.write(SCOPE_TRANSIENT)
  e_in.write(SCOPE_RESET)

e_out.write(SCOPE_RESET)
#transient_init()

from itertools import *
from collections import *

import matplotlib
matplotlib.use("Qt4Agg")
from matplotlib.animation import *
import matplotlib.pyplot as plt

fig = plt.figure()
lim = SCOPE_DATA_SIZE / 2
ax = plt.axes(xlim=(0, lim), ylim=(-1.2, 1.2))
lines = [ax.plot([], [], color="g")[0], ax.plot([], [], color="r")[0]]
x = range(0, lim)
ys = [deque(maxlen=lim), deque(maxlen=lim)]

def wait_trigger():
  e_out.write(SCOPE_TRIGGER)
  b = chr(e_in.read(1)[0])
  if (b != SCOPE_READY) and (b != SCOPE_WAITING):
    e_in.read(63) # read junk data
  while b != SCOPE_READY:
    b = chr(e_in.read(1)[0])

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

def transient_recorder(i):
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
