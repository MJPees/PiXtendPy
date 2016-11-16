#!/usr/bin/python3
########################################################
''' little demo...
- read version of firmware
- set Relay 1 on and of
- read temperature and humidity from DHT11
- swich all digital out as running light
- watch on changes at digital in
- switch all relays if one digital in is set to high
- toggle analog out
- Write "Hello World" to Serialport
- reads messages from serialport (ping/pong)
'''
########################################################

from piXtend import *

i = 0
digitalInput = 255
humidity = 0

def Spi_SetupAll():
    Spi_Setup(0)
    Spi_Setup(1)

def GetPixData():
    global i
    global digitalInput
    global humidity
    global temp
    dht1Temp = InputData['rTemp1']
    dht1Hum = InputData['wHumid1']
    #Only print out valid data
    if( dht1Hum > 0 and dht1Hum != humidity):
        #Write out the data on the linux console
        print("Temperature [°C]: %.2f" % dht1Temp)
        print("Humidity [%%]: %.2f" % (dht1Hum / 10))
        humidity = dht1Hum
    
    if i > 0xFF:
        i = 0
    else:
        i += 1 
    OutputData['byDigOut'] = i
    byDigIn = InputData['byDigIn']
    if byDigIn != digitalInput:
        print("current digital input: %s" % byDigIn)
        digitalInput = byDigIn
    return(0)


if __name__ == '__main__':
    try:
        Spi_SetupAll()
        Spi_Set_Relays(1)
        print('firmware: ' + Get_Hw_Version())
        time.sleep(1)

        SerialPort_Open('/dev/ttyAMA0',115200)
        SerialPort_Write_Text('Hello World\r\n')

        OutputData['byUcCtrl'] = 0x11 # Set the PiXtend-Controller to RUN mode (16) plus watchdog (1)
        OutputData['byGpioCtrl'] = 0x20 # GPIO1 is used for DHT1 (DHT22)
        OutputDataDAC['wAOut0'] = 1023 # pre-load analog outputs
        OutputDataDAC['wAOut1'] = 1023
        while True:
            #Do the data transfer!
            Spi_AutoMode()
            Spi_AutoModeDAC()
            #Set all Relays if at least one digital input is High
            if InputData['byDigIn'] > 0:
                print("Input detected - setting Relays!")
                OutputData['byRelayOut'] = 15
            else:
                OutputData['byRelayOut'] = 0
            #Toggle Analog Outputs
            if (OutputDataDAC['wAOut0'] == 1023) or (OutputDataDAC['wAOut1'] == 1023):
                OutputDataDAC['wAOut0'] = 512
                OutputDataDAC['wAOut1'] = 512
            else:
                OutputDataDAC['wAOut0'] = 1023
                OutputDataDAC['wAOut1'] = 1023
            #Read data and print it out
            GetPixData()
            
            serial_in = SerialPort_Read_Text()
            if serial_in != "":
                message = 'got serial message: ' + serial_in
                print(message)
                SerialPort_Write_Text(message + '\r\n')
            
            #Take a 250 ms break between the cycles (min. 100 ms)
            time.sleep(0.250)
         
    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt")
    except Exception as e:
        print("error : " + str(e))
    finally:
        SerialPort_Close()