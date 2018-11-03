#!/usr/bin/python

from time import sleep
import smbus

# Endereco do Arduino
ARDUINO = 0x42

# Objeto para acesso ao I2C1
bus = smbus.SMBus(1)

# Laco principal
while True:
  sleep(.3)
  resp = bus.read_i2c_block_data(ARDUINO, 0)
  val = resp[0]*256+resp[1]
  rgb=[]
  if val < 256:
    for i in range(7):
      rgb.append(val)
      rgb.append(0)
      rgb.append(0)
  elif val < 512:
    for i in range(7):
      rgb.append(511-val)
      rgb.append(val-256)
      rgb.append(0)
  elif val < 768:
    for i in range(7):
      rgb.append(0)
      rgb.append(767-val)
      rgb.append(val-512)
  else:
    for i in range(7):
      rgb.append(val-768)
      rgb.append(val-768)
      rgb.append(255)
  bus.write_i2c_block_data(ARDUINO, 1, rgb)
