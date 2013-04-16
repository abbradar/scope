#!/usr/bin/env python2

f = open('fw.hex', 'r')
o = open('fw.bin', 'wb')
for line in f:
  broken = line.split(' ')
  broken = broken[broken.index('') + 1:]
  broken = broken[:broken.index('')]
  for char in broken:
    b = chr(int(char, base=16))
    o.write(b)
o.close()