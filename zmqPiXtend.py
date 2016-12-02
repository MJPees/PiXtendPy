#!/usr/bin/python3
########################################################
'''
'''
########################################################

from piXtend import *
import time
import zmq
import datetime
import random
import json
import threading
import queue

##########################################################################################
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--showHint",help="show hint",action="store_true")
parser.add_argument("--showDescription",help="show description",action="store_true")
parser.add_argument("-ip",default="192.168.0.129",help="IP of ZMQ-server",type=str)
parser.add_argument("-hwip",default="192.168.0.33",help="IP of PiXtend",type=str)
args = parser.parse_args()

def showHint():
    print('PiXtend at ip-address ' + args.hwip)

def showDescription():
    print('Valid commands:')
    print('hwSet("' + args.hwip + '","byDigOut",0,0xFF)') # set all digital out to high
    print('hwSet("' + args.hwip + '","byDigOut",0,0x01)') # set all digital out to low excep pin1
    print('hwSet("' + args.hwip + '","byDigOut",1,0)') # set digital pin1 to low
    print('hwSet("' + args.hwip + '","byRelayOut",0,0x00)') # set all relays to low
    print('hwSet("' + args.hwip + '","byRelayOut",1,1)') # set relay 1 to high
    print('hwSet("' + args.hwip + '","wPwm0",0x255)') # set servo 0 to 255
    print('hwSet("' + args.hwip + '","wPwm1",128)') # set servo 1 to 128
    print('hwSet("' + args.hwip + '","wAOut0",0,1023)') # set analog out0  to 1023 (max)
    print('hwSet("' + args.hwip + '","wAOut1",0,0)') # set analog out1  to 0


if args.showHint:
    showHint()
    exit()

if args.showDescription:
    showDescription()
    exit()

serverIP = args.ip
###########################################################################################

context = zmq.Context()
zmqSender = context.socket(zmq.PUB)
zmqSender.connect('tcp://' + serverIP + ':5559')

zmqReceiver = context.socket(zmq.SUB)
zmqReceiver.connect('tcp://' + serverIP + ':6560')
topicfilter = args.hwip
zmqReceiver.setsockopt_string(zmq.SUBSCRIBE, topicfilter)

OutputData['byUcCtrl'] = 0x11 # Set the PiXtend-Controller to RUN mode (16) plus watchdog (1)

data_queue = queue.Queue()

def Spi_SetupAll():
    Spi_Setup(0)
    Spi_Setup(1)

def Start_PiXtend_AutoLoop():
    Spi_SetupAll()
    inState = {}
    outState = {}
    # last_time needed for cycletime
    last_time = datetime.datetime.now()
    while True:
        try:
            data = data_queue.get(True, 0.02) # non blocking
            if data['pin'] == '':
                data['pin'] = 0 
            data['pin'] = int(data['pin'])
            data['value'] = int(data['value'])
            if data['cmd'] == 'wAOut0' or data['cmd'] == 'wAOut1':
                OutputDataDAC[data['cmd']] = data['value']
            else:
                if data['pin'] == 0:
                    OutputData[data['cmd']] = data['value']
                    out_pin = 0
                else:
                    out_pin = data['pin'] - 1
                if data['value'] > 0:
                    OutputData[data['cmd']] |= (1 << out_pin)
                else:
                    OutputData[data['cmd']] &= (0xFF - (out_pin + 1))
        except queue.Empty:
            data = {'timestamp':'2016-12-2 23:10:0.0'}
        try:
            # cycletime minimum = 25 ms
            while datetime.datetime.now() - datetime.timedelta(microseconds=25000) < last_time:
                time.sleep(0.001)
            #Do the data transfer!
            Spi_AutoMode()
            Spi_AutoModeDAC()
            last_time = datetime.datetime.now()
            if last_time > datetime.datetime.strptime(data['timestamp'],"%Y-%m-%d %H:%M:%S.%f"):
                data['timestamp'] = str(last_time)
            for key in InputData.keys():
                if key in inState:
                    if inState[key] != InputData[key]:
                        data['cmd'] = key
                        data['value'] = InputData[key]
                        zmqSender.send_multipart([args.hwip, json.dumps(data).encode('UTF-8')])
                    inState[key] = InputData[key]
            for key in OutputData.keys():
                if key in outState:
                    if outState[key] != OutputData[key]:
                        zmqSender.send_multipart([args.hwip.encode('utf-8'), json.dumps(data).encode('UTF-8')])
                outState[key] = OutputData[key]
            for key in OutputDataDAC.keys():
                if key in outState:
                    if outState[key] != OutputDataDAC[key]:
                        zmqSender.send_multipart([args.hwip, json.dumps(data).encode('UTF-8')])
                    outState[key] = OutputDataDAC[key]
        except Exception as e:
            print("error : " + str(e))

if __name__ == '__main__':
    try:
        piXtendThread = threading.Thread(target=Start_PiXtend_AutoLoop)
        piXtendThread.daemon = True
        piXtendThread.start()
        while True:
            #wait for zmq-messages
            topic, message = [x.decode("utf-8") for x in zmqReceiver.recv_multipart()]
            #print(topic)
            #print(message)
            if topic == args.hwip:
                data = json.loads(message)
                data_queue.put(data)
    except KeyboardInterrupt:
            print("Caught KeyboardInterrupt, terminating processes")
    except Exception as e:
        print("error : " + str(e))
