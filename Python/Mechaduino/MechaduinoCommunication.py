# -*- coding: utf-8 -*-
"""
Created on Fri May  5 00:08:33 2017

@author: hoogendijkta
"""

#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Update a simple plot as rapidly as possible to measure speed.
"""
import serial
import struct
from pylab import *
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
from pyqtgraph import ptime
import time
import sys
import cPickle as pickle
import time

class StepResponseData():
    def __init__(self):
        self.pPGain = None
        self.pIGain = None
        self.pDGain = None 
        self.pLPF = None
        self.vPGain = None
        self.vIGain = None
        self.vDGain = None
        self.vLPF = None
        self.Fs = None
        self.mode = None                         
        self.Torque = []
        self.Velocity = []
        self.Position = []
        self.Setpoint = []
        self.Error = []
        self.Version = 'MechaduinoData V0.1'
        self.TimeStamp = time.time()


class MechaduinoCommunication(pg.QtCore.QThread):
    newData = pg.QtCore.Signal()
    
    def __init__(self, PortName='/dev/ttyACM0', Baudrate=115200, max_buffer_size=40000):
        super(MechaduinoCommunication, self).__init__()
        
        try:
            self.serialCOMM = serial.Serial(
                port=PortName,\
                baudrate=Baudrate,\
                parity=serial.PARITY_NONE,\
                stopbits=serial.STOPBITS_ONE,\
                bytesize=serial.EIGHTBITS,\
                timeout=1)
        except:
           self.serialCOMM = None
#           print "Could not create serial port"+ str(PortName) 
        
 
        if self.serialCOMM:
            self.serialCOMM.flushInput()
            self.serialCOMM.flushOutput()
            self.max_buffer_size = max_buffer_size
            self.UnpackedData = ''
            self._isRunning=False
            self._isStopped=True;      
            self.Data = StepResponseData()
            
#            print("connected to: " + self.serialCOMM.portstr)
    
    def run(self):
        if self.serialCOMM:
            Buffer = '';
            self._isRunning=True;
            self._isStopped=False;
            EndToken = ":END:"
            StartToken = ":START:"
            while self._isRunning:
                Char = self.serialCOMM.read(10);
                if Char:
                    Buffer += Char 
                    
                    #if the buffer contains start and end symbols extract the data
                    START = Buffer.find(StartToken)
                    END = Buffer.find(EndToken, START)
#                    print "START"+str(START)+" END"+str(END)
                    if (END != -1) and (START != -1):   
                        if  (END>START):
                            Packet = Buffer[START:END+len(EndToken)]
                            Buffer = Buffer[END+len(EndToken):]
                        else:
                            Buffer = ''
                        #Size = self.__ExtractSize__(Packet)

                        UnpackedData = self.__ExtractData__(Packet)
                        #Basic check if size corresponds
                        #TODO create checksum 
                        if UnpackedData:                    
                            self.Data = UnpackedData
                            self.newData.emit()                          

                    #if the buffer is bigger than the max buffer size clear buffer    
                    elif len(Buffer) > self.max_buffer_size:                      
                        Buffer = ''
                    
        self._isStopped=True;
                        
    def __ExtractData__(self, Packet):
        Data = StepResponseData()
        
        Packet = Packet.split("\r\n")
        FloatData = [float(x) for x in Packet[1].split(' ')[0:-1]]

        Data.pPGain = FloatData[0]
        Data.pIGain = FloatData[1]
        Data.pDGain = FloatData[2] 
        Data.pLPF = FloatData[3]
        Data.vPGain = FloatData[4]
        Data.vIGain = FloatData[5]
        Data.vDGain = FloatData[6]
        Data.vLPF = FloatData[7]
        Data.Fs = FloatData[8]
        Data.mode = Packet[1].split(' ')[-1]
        
        for line in Packet[2:-1]:
            FloatData = [float(x) for x in line.split(' ')]
            Data.Velocity.append(FloatData[2])
            Data.Position.append(FloatData[1])
            Data.Torque.append(FloatData[0]/100.0) 
            if Data.mode=='x':
                Data.Error.append(FloatData[1]-FloatData[3])
                Data.Setpoint.append(FloatData[3])
                
            if Data.mode=='v':
                Data.Error.append(FloatData[2]-FloatData[3])
                Data.Setpoint.append(FloatData[3])

            if Data.mode=='t':
                Data.Error.append((FloatData[0]-FloatData[3])/100)
                Data.Setpoint.append(FloatData[3]/100)
                
        return Data              
       
    def stop(self):
        time.sleep(0.1)
        self._isRunning = False
        while not self._isStopped:
            time.sleep(0.05)
            
        if self.serialCOMM:     
            self.serialCOMM.close()
#        QtGui.QApplication.instance().quit()        

    def getData(self):
        return self.Data

       
    def __del__(self):
        if self.serialCOMM:     
            self.serialCOMM.close()
    
    def sendData(self, writeData):
        if self.serialCOMM:     
            self.serialCOMM.write(writeData)
        else:
            print "error no open serial port"


class MechaduinoDatalogger():
    def __init__(self):
        self.Data = []
        self.Version = 'Mechaduino datalogger V0.1'
        
    def AddItem(self, item):       
        self.Data.append(item)
    
    def SaveData(self, FileName):
        with open(FileName, 'wb') as output:
              pickle.dump(self.Version, output, pickle.HIGHEST_PROTOCOL)
              pickle.dump(self.Data, output, pickle.HIGHEST_PROTOCOL)

    def OpenData(self, FileName):
        with open(FileName, 'rb') as input:
              self.Version = pickle.load(input)
              self.Data = pickle.load(input) 
