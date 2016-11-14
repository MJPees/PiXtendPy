#!/usr/bin/python3
"""
# Copyright (C) 2016 Marc Pees
# l-n-s.de, Im großen Boden 2
# 56589 Niederbreitbach, Germany
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import time
import wiringpi
byInitFlag = 0

OutputDataDAC = {'wAOut0' : 0, 'wAOut1' : 0}

OutputData = {}
OutputData['byDigOut'] = 0
OutputData['byRelayOut'] = 0
OutputData['byGpioOut'] = 0
OutputData['wPwm0'] = 0
OutputData['wPwm1'] = 0
OutputData['byPwm0Ctrl0'] = 0
OutputData['byPwm0Ctrl1'] = 0
OutputData['byPwm0Ctrl2'] = 0
OutputData['byGpioCtrl'] = 0
OutputData['byUcCtrl'] = 0
OutputData['byAiCtrl0'] = 0
OutputData['byAiCtrl1'] = 0
OutputData['byPiStatus'] = 0
OutputData['byAux0'] = 0

InputData = {}
InputData['byDigIn'] = 0
InputData['wAi0'] = 0
InputData['wAi1'] = 0
InputData['wAi2'] = 0
InputData['wAi3'] = 0
InputData['byGpioIn'] = 0
InputData['wTemp0'] = 0
InputData['wTemp1'] = 0
InputData['wTemp2'] = 0
InputData['wTemp3'] = 0
InputData['wHumid0'] = 0
InputData['wHumid1'] = 0
InputData['wHumid2'] = 0
InputData['wHumid3'] = 0
InputData['byUcVersionL'] = 0
InputData['byUcVersionH'] = 0
InputData['byUcStatus'] = 0
InputData['rAi0'] = 0
InputData['rAi1'] = 0
InputData['rAi2'] = 0
InputData['rAi3'] = 0
InputData['rTemp0'] = 0
InputData['rTemp1'] = 0
InputData['rTemp2'] = 0
InputData['rTemp3'] = 0
InputData['rHumid0'] = 0
InputData['rHumid1'] = 0
InputData['rHumid2'] = 0
InputData['rHumid3'] = 0


def getHwVersion():
  version = Spi_Get_uC_Version()
  versionH = version >> 8
  versionL = version & 0xFF
  print("%s.%s" % (versionH, versionL))
  return(version)


def limit(value, min_value = 0, max_value = 255):
  if value < min_value:
    return min_value
  elif value > max_value:
    return max_value
  else: 
    return value


def crc16_calc(crc, data):
  crc = crc ^ data
  for i in range(8):
    if crc & 1:
      crc = (crc >> 1) ^ 0xA001
    else:
      crc = (crc >> 1)
  return(crc)


def Spi_AutoModeDAC():
  Spi_Set_Aout(0, OutputDataDAC['wAOut0'])
  Spi_Set_Aout(1, OutputDataDAC['wAOut1'])
  return(0)


def Spi_AutoMode():
  spi_output = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
  spi_device = 0
  spi_output[0] = 128
  spi_output[1] = 255
  spi_output[2] = OutputData['byDigOut']
  spi_output[3] = OutputData['byRelayOut']
  spi_output[4] = OutputData['byGpioOut']
  spi_output[5] = OutputData['wPwm0'] & 0xFF
  spi_output[6] = (OutputData['wPwm0'] >> 8) & 0xFF
  spi_output[7] = OutputData['wPwm1'] & 0xFF
  spi_output[8] = (OutputData['wPwm1'] >> 8) & 0xFF
  spi_output[9] = OutputData['byPwm0Ctrl0']
  spi_output[10] = OutputData['byPwm0Ctrl1']
  spi_output[11] = OutputData['byPwm0Ctrl2']
  spi_output[12] = OutputData['byGpioCtrl']
  spi_output[13] = OutputData['byUcCtrl']
  spi_output[14] = OutputData['byAiCtrl0']
  spi_output[15] = OutputData['byAiCtrl1']
  spi_output[16] = OutputData['byPiStatus']
  byAux0 = OutputData['byAux0']
  #Calculate CRC16 Transmit Checksum
  crcSum = 0xFFFF
  for i in range(2,31):
    crcSum = crc16_calc(crcSum, spi_output[i]);
  spi_output[31] = crcSum & 0xFF # CRC Low Byte
  spi_output[32] = crcSum >> 8 # CRC High Byte
  spi_output[33] = 128 # Termination
  byte_string = bytes(spi_output)

  #Initialise SPI Data Transfer with OutputData
  wiringpi.wiringPiSPIDataRW(spi_device, byte_string)

  #byte_string now contains all returned data, assign values to InputData
  InputData['byDigIn'] = byte_string[2];
  InputData['wAi0'] = (byte_string[4] << 8) | (byte_string[3])
  InputData['wAi1'] = (byte_string[6] << 8) | (byte_string[5])
  InputData['wAi2'] = (byte_string[8] << 8) | (byte_string[7])
  InputData['wAi3'] = (byte_string[10] << 8) | (byte_string[9])
  InputData['byGpioIn'] = byte_string[11]
  InputData['wTemp0'] = (byte_string[13] << 8) | (byte_string[12])
  InputData['wTemp1'] = (byte_string[15] << 8) | (byte_string[14])
  InputData['wTemp2'] = (byte_string[17] << 8) | (byte_string[16])
  InputData['wTemp3'] = (byte_string[19] << 8) | (byte_string[18])
  InputData['wHumid0'] = (byte_string[21] << 8) | (byte_string[20])
  InputData['wHumid1'] = (byte_string[23] << 8) | (byte_string[22])
  InputData['wHumid2'] = (byte_string[25] << 8) | (byte_string[24])
  InputData['wHumid3'] = (byte_string[27] << 8) | (byte_string[26])
  InputData['byUcVersionL'] = byte_string[28]
  InputData['byUcVersionH'] = byte_string[29]
  InputData['byUcStatus'] = byte_string[30]
  if byAux0 & (0b00000001):
    InputData['rAi0'] = InputData['wAi0'] * (10.0 / 1024)
  else:
    InputData['rAi0'] = InputData['wAi0'] * (5.0 / 1024)
  if byAux0 & (0b00000010):
    InputData['rAi1'] = InputData['wAi1'] * (10.0 / 1024)
  else:
    InputData['rAi1'] = InputData['wAi1'] * (5.0 / 1024)
  InputData['rAi2'] = InputData['wAi2'] * 0.024194115990990990990990990991;
  InputData['rAi3'] = InputData['wAi3'] * 0.024194115990990990990990990991;
  InputData['rTemp0'] = InputData['wTemp0'] / 10.0;
  InputData['rTemp1'] = InputData['wTemp1'] / 10.0;
  InputData['rTemp2'] = InputData['wTemp2'] / 10.0;
  InputData['rTemp3'] = InputData['wTemp3'] / 10.0;
  InputData['rHumid0'] = InputData['wHumid0'] / 10.0;
  InputData['rHumid1'] = InputData['wHumid1'] / 10.0;
  InputData['rHumid2'] = InputData['wHumid2'] / 10.0;
  InputData['rHumid3'] = InputData['wHumid3'] / 10.0;
  #Calculate CRC16 Receive Checksum
  crcSum = 0xFFFF
  for i in range(2,31): 
    crcSum = crc16_calc(crcSum, byte_string[i]);
  crcSumRx = (byte_string[32]<<8) + byte_string[31];
  if crcSumRx != crcSum:
    return(-1)
  else:
    return(0)


def Spi_Set_Dout(value):
  value = limit(value,0,255)
  spi_output = [0,0,0,0]
  spi_device = 0
  spi_output[0] = 0b10101010
  spi_output[1] = 0b00000001
  spi_output[2] = value
  spi_output[3] = 0b10101010
  wiringpi.wiringPiSPIDataRW(spi_device, bytes(spi_output))
  return(0)


def Spi_Get_Dout():
  spi_output = [0,0,0,0]
  spi_device = 0
  spi_output[0] = 0b10101010 # Handshake - begin
  spi_output[1] = 0b00010010 # Command 18
  spi_output[2] = 0b10101010 # readback command 
  spi_output[3] = 0b10101010 # read value
  byte_string = bytes(spi_output)
  wiringpi.wiringPiSPIDataRW(spi_device, byte_string)
  time.sleep(0.01)
  return(byte_string[3])


def Spi_Get_Din():
  spi_output = [0,0,0,0]
  spi_device = 0
  spi_output[0] = 0b10101010 # Handshake - begin
  spi_output[1] = 0b00000010 # Command
  spi_output[2] = 0b10101010 # readback command
  spi_output[3] = 0b10101010 # read value
  byte_string = bytes(spi_output)
  wiringpi.wiringPiSPIDataRW(spi_device, byte_string)
  time.sleep(0.01)
  return(byte_string[3])


def Spi_Get_Ain(Idx):
  spi_output = [0,0,0,0,0]
  spi_device = 0
  for i in range(2):
    if Idx == 0 :
      spi_output[1] = 0b00000011 # Command
    elif Idx == 1:
      spi_output[1]=0b00000100 # Command
    elif Idx == 2:
      spi_output[1] = 0b00000101 # Command
    else:
      spi_output[1] = 0b00000110 # Command
    spi_output[0] = 0b10101010 # Handshake - begin
    spi_output[2] = 0b00000000 # readback command
    spi_output[3] = 0b00000000 # read value low
    spi_output[4] = 0b00000000 # read value high
    byte_string = bytes(spi_output)
    wiringpi.wiringPiSPIDataRW(spi_device, byte_string)
    time.sleep(0.1)
  high = byte_string[3]
  low = byte_string[4] << 8
  output = high | low
  return(output)


def Spi_Set_Aout(channel, value):
  spi_output = [0,0]
  spi_device = 1
  spi_output[0] = 0b00010000
  if channel == 1:
    spi_output[0] = spi_output[0] | 0b10000000
  if value > 1023:
    value=1023
  tmp = value & 0b1111000000
  tmp = tmp >> 6
  spi_output[0] = spi_output[0] | tmp
  tmp = value & 0b0000111111;
  tmp = tmp << 2;
  spi_output[1] = tmp
  byte_string = bytes(spi_output)
  wiringpi.wiringPiSPIDataRW(spi_device, byte_string)
  return(0)


def Spi_Set_Relays(value):
  value = limit(value,0,255)
  spi_output = [0,0,0,0]
  spi_device = 0
  spi_output[0] = 0b10101010
  spi_output[1] = 0b00000111
  spi_output[2] = value
  spi_output[3] = 0b10101010
  wiringpi.wiringPiSPIDataRW(spi_device,bytes(spi_output))
  return(0)


def Spi_Get_Relays():
  spi_output = [0,0,0,0]
  spi_device = 0
  spi_output[0] = 0b10101010
  spi_output[1] = 0b00010011
  spi_output[2] = 0b10101010
  spi_output[3] = 0b10101010
  byte_string = bytes(spi_output)
  wiringpi.wiringPiSPIDataRW(spi_device, byte_string)
  time.sleep(0.01)
  return(byte_string[3])


def Spi_Get_Temp(Idx):
  spi_output = [0,0,0,0,0]
  spi_device = 0
  for i in range(2):
    if Idx == 0:
      spi_output[1] = 0b00001010 # Command
    elif Idx == 1:
      spi_output[1]=0b00001011 # Command
    elif Idx == 2:
      spi_output[1] = 0b00001100 # Command
    else:
      spi_output[1] = 0b00001101 # Command
    spi_output[0] = 0b10101010 # Handshake - begin
    spi_output[2] = 0b00000000 # readback command
    spi_output[3] = 0b00000000 # read value low
    spi_output[4] = 0b00000000 # read value high
    byte_string = bytes(spi_output)
    wiringpi.wiringPiSPIDataRW(spi_device, byte_string)
    time.sleep(0.1)
  high = byte_string[3]
  low = byte_string[4] << 8
  output = high | low
  return(output)


def Spi_Get_Hum(Idx):
  spi_output = [0,0,0,0,0]
  spi_device = 0
  for i in range(2):
    if Idx == 0:
      spi_output[1] = 0b00001110 # Command
    elif Idx == 1:
      spi_output[1] = 0b00001111 # Command
    elif Idx == 2:
      spi_output[1] = 0b00010000 # Command
    else:
      spi_output[1] = 0b00010001 # Command
    spi_output[0] = 0b10101010 # Handshake - begin
    spi_output[2] = 0b00000000 # readback command
    spi_output[3] = 0b00000000 # read value low
    spi_output[4] = 0b00000000 # read value high
    byte_string = bytes(spi_output)
    wiringpi.wiringPiSPIDataRW(spi_device, byte_string)
    time.sleep(0.1)
  high = byte_string[3]
  low = byte_string[4] << 8
  output = high | low
  return(output)


def Spi_Set_Servo(channel, value):
  spi_output = [0,0,0,0]
  spi_device = 0
  if channel > 0:
    spi_output[1] = 0b10000001 # Command
  else:
    spi_output[1] = 0b10000000 # Command
  spi_output[0] = 0b10101010 # Handshake - begin
  spi_output[2] = value
  spi_output[3] = 0b10101010
  byte_string = bytes(spi_output)
  wiringpi.wiringPiSPIDataRW(spi_device,byte_string)
  return(0) 


def Spi_Set_Pwm(channel, value):
  spi_output = [0,0,0,0,0]
  spi_device = 0
  if channel > 0:
    spi_output[1] = 0b10000011 # Command
  else:
    spi_output[1] = 0b10000010 # Command
  spi_output[0] = 0b10101010 # Handshake - begin
  spi_output[2] = value & 0b0000000011111111
  spi_output[3] = (value & 0b1111111100000000) >> 8
  spi_output[4] = 0b10101010
  byte_string = bytes(spi_output)
  wiringpi.wiringPiSPIDataRW(spi_device,byte_string)
  return(0) 


def Spi_Set_PwmControl(value0, value1, value2):
  spi_output = [0,0,0,0,0,0]
  spi_device = 0
  spi_output[0] = 0b10101010 # Handshake - begin
  spi_output[1] = 0b10000100 # Command
  spi_output[2] = value0
  spi_output[3] = value1
  spi_output[4] = value2
  spi_output[5] = 0b10101010
  byte_string = bytes(spi_output)
  wiringpi.wiringPiSPIDataRW(spi_device,byte_string)
  return(0) 


def Spi_Set_GpioControl(value):
  spi_output = [0,0,0,0]
  spi_device = 0
  spi_output[0] = 0b10101010 # Handshake - begin
  spi_output[1] = 0b10000101 # Command
  spi_output[2] = value 
  spi_output[3] = 0b10101010
  byte_string = bytes(spi_output)
  wiringpi.wiringPiSPIDataRW(spi_device,byte_string)
  return(0) 


def Spi_Set_UcControl(value):
  spi_output = [0,0,0,0]
  spi_device = 0
  spi_output[0] = 0b10101010 # Handshake - begin
  spi_output[1] = 0b10000110 # Command
  spi_output[2] = value 
  spi_output[3] = 0b10101010
  byte_string = bytes(spi_output)
  wiringpi.wiringPiSPIDataRW(spi_device,byte_string)
  return(0) 


def Spi_Set_AiControl(value0, value1):
  spi_output = [0,0,0,0,0]
  spi_device = 0
  spi_output[0] = 0b10101010 # Handshake - begin
  spi_output[1] = 0b10000111 # Command
  spi_output[2] = value0
  spi_output[3] = value1
  spi_output[4] = 0b10101010
  byte_string = bytes(spi_output)
  wiringpi.wiringPiSPIDataRW(spi_device,byte_string)
  return(0) 


def Spi_Set_RaspStat(value):
  spi_output = [0,0,0,0]
  spi_device = 0
  spi_output[0] = 0b10101010 # Handshake - begin
  spi_output[1] = 0b10001000 # Command
  spi_output[2] = value 
  spi_output[3] = 0b10101010
  byte_string = bytes(spi_output)
  wiringpi.wiringPiSPIDataRW(spi_device,byte_string)
  return(0) 


def Spi_Setup(spi_device):
  global byInitFlag 
  pin_Spi_enable = 5
  Spi_frequence = 100000;
  if byInitFlag < 1:
    wiringpi.wiringPiSetup()
    byInitFlag = 1
  wiringpi.pinMode(pin_Spi_enable, wiringpi.OUTPUT)
  wiringpi.digitalWrite(pin_Spi_enable,1)
  wiringpi.wiringPiSPISetup(spi_device, Spi_frequence)
  return(0)


def Spi_uC_Reset():
  pin_reset = 4
  wiringpi.wiringPiSetup()
  wiringpi.pinMode(pin_reset, wiringpi.OUTPUT)
  wiringpi.digitalWrite(pin_reset,1)
  time.sleep(1)
  wiringpi.digitalWrite(pin_reset,0)
  return(0)


def Spi_Get_uC_Status():
  spi_output = [0,0,0,0]
  spi_device = 0
  spi_output[0] = 0b10101010 # Handshake - begin
  spi_output[1] = 0b10001010 # Command
  spi_output[2] = 0b10101010 # readback command 
  spi_output[3] = 0b00000000 # read value
  byte_string = bytes(spi_output)
  wiringpi.wiringPiSPIDataRW(spi_device,byte_string)
  time.sleep(0.01)
  return(byte_string[3]) 


def Spi_Get_uC_Version():
  spi_output = [0,0,0,0,0]
  spi_device = 0
  spi_output[0] = 0b10101010 # Handshake - begin
  spi_output[1] = 0b10001001 # Command
  spi_output[2] = 0b00000000 # readback command 
  spi_output[3] = 0b00000000 # read value low
  spi_output[4] = 0b00000000 # read value high
  byte_string = bytes(spi_output)
  wiringpi.wiringPiSPIDataRW(spi_device,byte_string)
  time.sleep(0.1)
  high = byte_string[4] << 8
  low = byte_string[3]
  version = high | low
  return(version) 


def Change_Gpio_Mode(pin, mode):
  wiringpi.wiringPiSetup()
  if mode == 1:
    wiringpi.pinMode(pin,wiringpi.OUTPUT)
  else:
    wiringpi.pinMode(pin,wiringpi.INPUT)
  return(0)


def Change_Serial_Mode(mode):
  pin_serial = 1 #Pin 1 ^= GPIO18 
  wiringpi.wiringPiSetup()
  wiringpi.pinMode(pin_serial, wiringpi.OUTPUT)
  if mode == 1:
    wiringpi.digitalWrite(pin_serial, 1) # RS485
  else:
    wiringpi.digitalWrite(pin_serial, 0) # RS232
  return(0)


def Spi_Set_Gpio(value):
  value = limit(value,0,255) 
  spi_output = [0,0,0,0]
  spi_device = 0
  spi_output[0] = 0b10101010
  spi_output[1] = 0b00001000
  spi_output[2] = value
  spi_output[3] = 0b10101010
  wiringpi.wiringPiSPIDataRW(spi_device, bytes(spi_output))
  return(0)


def Spi_Get_Gpio():
  spi_output = [0,0,0,0]
  spi_device = 0
  spi_output[0] = 0b10101010 # Handshake - begin
  spi_output[1] = 0b00001001 # Command
  spi_output[2] = 0b10101010 # readback command
  spi_output[3] = 0b10101010 # read value
  byte_string = bytes(spi_output)
  wiringpi.wiringPiSPIDataRW(spi_device, byte_string)
  time.sleep(0.01)
  return(byte_string[3])