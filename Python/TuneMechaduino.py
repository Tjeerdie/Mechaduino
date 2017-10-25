# -*- coding: utf-8 -*-
"""
Created on Sat May  6 22:28:21 2017

@author: hoogendijkta
"""

#TODO
# Add open log windows with open file selection
# Show current logging bin and make rest of the background white
# Display logged number of items per bin in logging window instead of timing table
# Make live updating graph with last x received data 
# Start on log analizing tools (plotting, fft. set correct degrees under the graph)



import pyqtgraph as pg
import sys
import glob
import serial
import xml.etree.ElementTree
import datetime
from PyQt4.QtGui import *
from PyQt4 import uic
from PyQt4.QtCore import *
from xml.dom import minidom
import numpy as np
from Mechaduino.MechaduinoCommunication import *
  
class PlotData(pg.PlotWidget):
    def __init__(self):
        super(PlotData, self).__init__()
        self = pg.plot()
        self.curve = self.plot()       
        self.lastTime = ptime.time()
        self.fps = None
        self.update()
        
#    def update(self, data=[]):
##        global curve, p, lastTime, fps, data
#        self.textBrowserComStatus.append("Updated graph")
#        self.curve.setData(data)
#        self.now = ptime.time()
#        dt = self.now - self.lastTime
#        self.lastTime = self.now
#        if self.fps is None:
#            self.fps = 1.0/dt
#        else:
#            s = np.clip(dt*3., 0, 1)
#            self.fps = self.fps * (1-s) + (1.0/dt) * s
#            
#        self.setTitle('%0.2f fps' % self.fps)
        
    def closeEvent(self, event):
        self.textBrowserComStatus.append("Closing")
        del(self) #.close() 
        
        
#        self.close()  
#        app.processEvents()  ## force complete redraw for every plot

#        MechaduinoCommunication.newData.connect(update)

class Ui_MechaduinoMainWindow(QtGui.QMainWindow):
    
    def __init__(self):
        super(Ui_MechaduinoMainWindow, self).__init__()
        uic.loadUi('ui/MechaduinoMainWindow.ui', self)  
#        self.tableWidgetIgnition.horizontalHeader().setResizeMode(QHeaderView.Stretch)
#        self.tableWidgetIgnition.verticalHeader().setResizeMode(QHeaderView.Stretch)
#        self.tableWidgetIgnition.verticalHeader().setVisible(True)
#        self.tableWidgetIgnition_2.horizontalHeader().setResizeMode(QHeaderView.Stretch)
#        self.tableWidgetIgnition_2.verticalHeader().setResizeMode(QHeaderView.Stretch)
#        self.tableWidgetIgnition_2.verticalHeader().setVisible(True)
#        self.tableWidgetIgnition_3.horizontalHeader().setResizeMode(QHeaderView.Stretch)
#        self.tableWidgetIgnition_3.verticalHeader().setResizeMode(QHeaderView.Stretch)
#        self.tableWidgetIgnition_3.verticalHeader().setVisible(True)
#        self.tableWidgetIgnition.verticalHeader().setResizeMode(QHeaderView.Stretch)                       
        self.PopulateSerialComboBox()
#        self.SaveLog.setEnabled(False)       
#        self.lcdNumberMap.display(0)
#        self.lcdNumberRPM.display(0)
        
#        self.LoadTable.clicked.connect(self.SetIgnitionTable)
        self.ConnectMechaduino.clicked.connect(self.ConnectWithMechaduino)
        self.do_pStep.clicked.connect(self.executepStep)
        self.do_vStep.clicked.connect(self.executevStep)
        self.do_tStep.clicked.connect(self.executetStep)
        
        self.setGainsvPID.clicked.connect(self.SendGainsvPID)
        self.setGainspPID.clicked.connect(self.SendGainspPID)
        
#        self.LiveView.clicked.connect(self.OpenLivePlot)
        self.textBrowserComStatus.append("Mechaduino tuning software V0.1")
        
        self.pcurve = self.GraphPosition.plot()
        self.vcurve = self.GraphVelocity.plot()
        self.tcurve = self.GraphTorque.plot()
        self.ecurve = self.GraphError.plot()

        
        ### make plot window pretty        
        self.GraphPosition.setRange(QtCore.QRectF(0, -180, 0.2, 360)) 
        self.GraphPosition.setLabel('bottom', 'Time', units='Seconds')
        self.GraphPosition.setLabel('left', 'Position', units='Degree')
        
        self.GraphVelocity.setRange(QtCore.QRectF(0, -800, 0.2, 1600)) 
        self.GraphVelocity.setLabel('bottom', 'Time', units='Seconds')
        self.GraphVelocity.setLabel('left', 'Speed', units='RPM')
        
        self.GraphTorque.setRange(QtCore.QRectF(0, -2000, 0.2, 4000)) 
        self.GraphTorque.setLabel('bottom', 'Time', units='Seconds')
        self.GraphTorque.setLabel('left', 'Current', units='A')
        
        self.GraphError.setRange(QtCore.QRectF(0, -1, 0.2, 2)) 
               
        self.GraphPosition.setTitle('Position') 
        self.GraphVelocity.setTitle('Velocity') 
        self.GraphTorque.setTitle('Torque') 
        self.GraphError.setTitle('Tracking error') 
        
        
        self.do_pStep.setEnabled(False)
        self.do_vStep.setEnabled(False)
        self.do_tStep.setEnabled(False)
        
        self.setGainsvPID.setEnabled(False)
        self.setGainspPID.setEnabled(False)
               
#        self.update()
        
    def executepStep(self):
        #set step amplitude
        self.MechaduinoCommunication.sendData("ja"+ str(self.pStepAmplitude.value()) + "\r\n")   
   
        #set step start
        self.MechaduinoCommunication.sendData("js"+ str(self.pStepStart.value()) + "\r\n")           
  
        #set step end
        self.MechaduinoCommunication.sendData("je"+ str(self.pStepEnd.value()) + "\r\n")     
        
        # set mode to position control and start close loop mode
        self.MechaduinoCommunication.sendData("xy\r\n")   
        
        #execute step
        self.MechaduinoCommunication.sendData("jx\r\n") 

     #        self.comboBoxSerialPort.activated.connect(self.PopulateSerialComboBox)

    def executevStep(self):
        #set step amplitude
        self.MechaduinoCommunication.sendData("ja"+ str(self.vStepAmplitude.value()) + "\r\n")   
   
        #set step start
        self.MechaduinoCommunication.sendData("js"+ str(self.vStepStart.value()) + "\r\n")           
  
        #set step end
        self.MechaduinoCommunication.sendData("je"+ str(self.vStepEnd.value()) + "\r\n")     
        
        # set mode to position control and start close loop mode
        self.MechaduinoCommunication.sendData("vy\r\n")   
        
        #execute step
        self.MechaduinoCommunication.sendData("jx\r\n") 
        
    def executetStep(self):
        #set step amplitude
        self.MechaduinoCommunication.sendData("ja"+ str(self.tStepAmplitude.value()) + "\r\n")   
   
        #set step start
        self.MechaduinoCommunication.sendData("js"+ str(self.tStepStart.value()) + "\r\n")           
  
        #set step end
        self.MechaduinoCommunication.sendData("je"+ str(self.tStepEnd.value()) + "\r\n")     
        
        # set mode to position control and start close loop mode
        self.MechaduinoCommunication.sendData("ty\r\n")   
        
        #execute step
        self.MechaduinoCommunication.sendData("jx\r\n") 

     #        self.comboBoxSerialPort.activated.connect(self.PopulateSerialComboBox)
    def SendGainsvPID(self):
        #set vPID gains
        self.MechaduinoCommunication.sendData("jgvp"+ str(self.vKpGain.value()) + "\r\n")   
        self.MechaduinoCommunication.sendData("jgvi"+ str(self.vKiGain.value()) + "\r\n")   
        self.MechaduinoCommunication.sendData("jgvd"+ str(self.vKdGain.value()) + "\r\n")           
        
    def SendGainspPID(self):
        #set pPID gains
        self.MechaduinoCommunication.sendData("jgpp"+ str(self.pKpGain.value()) + "\r\n")   
        self.MechaduinoCommunication.sendData("jgpi"+ str(self.pKiGain.value()) + "\r\n")   
        self.MechaduinoCommunication.sendData("jgpd"+ str(self.pKdGain.value()) + "\r\n")   
        
    def UpdateLivePlot(self, data):
        TimeSeries = np.arange(0.0, (float(len(data.Position)))/data.Fs, 1/data.Fs)
        
        self.GraphTorque.setRange(QtCore.QRectF(0, -2.0, TimeSeries[-1], 4.0)) 
        self.GraphVelocity.setRange(QtCore.QRectF(0, -1000, TimeSeries[-1], 2000)) 
        self.GraphPosition.setRange(QtCore.QRectF(0, min(data.Position), TimeSeries[-1], max(data.Position)-min(data.Position))) 
        self.GraphError.setRange(QtCore.QRectF(0, min(data.Error), TimeSeries[-1], max(data.Error)-min(data.Error))) 
        
        self.GraphPosition.clear()
        self.GraphPosition.plot(y=data.Position, x=TimeSeries)
        if data.mode == 'x':
            self.GraphPosition.plot(y=data.Setpoint, x=TimeSeries, pen='g')
        
        self.GraphVelocity.clear()
        self.GraphVelocity.plot(y=data.Velocity, x=TimeSeries)
        if data.mode == 'v':
            self.GraphVelocity.plot(y=data.Setpoint, x=TimeSeries, pen='g')
            
        self.GraphTorque.clear()
        self.GraphTorque.plot(y=data.Torque, x=TimeSeries)
        if data.mode == 't':
            self.GraphTorque.plot(y=data.Setpoint, x=TimeSeries, pen='g')    


        self.ecurve.setData(y=data.Error, x=TimeSeries)
        
#        self.LiveGraphWidget.setTitle('%0.2f fps' % self.fps)        
#        self.LiveGraphWidget.setRange(QtCore.QRectF(0, 200, 9000, 2200)) 
        
#    def OpenLivePlot(self):
#        self.LiveGraph = PlotData()
#        self.LiveView.clicked.disconnect(self.OpenLivePlot)
#        self.LiveView.clicked.connect(self.CloseLivePlot)
#        self.LiveView.clicked.connect( self.LiveGraph.close)
#    
#    def CloseLivePlot(self):
#        self.LiveGraph.close()
#        self.LiveGraph = None
#        self.LiveView.clicked.disconnect(self.CloseLivePlot)
#        self.LiveView.clicked.connect(self.OpenLivePlot)    

    def SavelogDialogBox(self):
        if self.IonLogger:
            try:
                fname = QFileDialog.getSaveFileName(None, 'Save mechaduino step repsonses', datetime.datetime.now().strftime("%Y%m%d_%H%M%S_") + 'MechaduinoLog' + ".mdl","Mechaduino logs (*.mdl)")
        #        fname = "/home/hoogendijkta/Documents/hobby/Ion sens/Python/ignitionTbl1_2017-05-07_00.48.31.table"
            except:
                #TODO return error that file does not load
                fname=None
            
            self.IonLogger.SaveData(fname)
    
#    def SetIgnitionTable(self):
#        try:
#            fname = QFileDialog.getOpenFileName(None, 'Open table', 'gh',"Table Files (*.table)")
#    #        fname = "/home/hoogendijkta/Documents/hobby/Ion sens/Python/ignitionTbl1_2017-05-07_00.48.31.table"
#        except:
#            #TODO return error that file does not load
#            fname=None
#                
#        if not fname:
#            self.xAxis = None
#            self.yAxis = None
#            self.zValues = None
#            return
#            
#        xmldoc = minidom.parse(str(fname))
#        
#        self.xAxis = [int(i) for i in xmldoc.getElementsByTagName('xAxis')[0].childNodes[0].data.split()]      
#        self.yAxis = [float(i) for i in xmldoc.getElementsByTagName('yAxis')[0].childNodes[0].data.split()]
#        self.rpmAxis = self.xAxis
#        self.mapAxis = self.yAxis
#        self.zValues = np.array([float(i) for i in xmldoc.getElementsByTagName('zValues')[0].childNodes[0].data.split()]).reshape(len(self.xAxis),len(self.yAxis))
#        
##        self.tableWidgetIgnition.setItem(0 , 0, QTableWidgetItem("wat een drama"))
#        count_i = 0;
#        count_j = 0;
#        for i in np.flipud(self.zValues):
#            count_j = 0;
#            for j in i:
#                self.tableWidgetIgnition.setItem(count_i , count_j, QTableWidgetItem(str(j)))
#                self.tableWidgetIgnition_2.setItem(count_i , count_j, QTableWidgetItem("0"))
#                self.tableWidgetIgnition_2.item(count_i, count_j).setBackground(QColor(255,0,0)) 
#                
#                self.tableWidgetIgnition_3.setItem(count_i , count_j, QTableWidgetItem("0"))
#                count_j += 1
#            count_i += 1
#        
#        self.tableWidgetIgnition.setHorizontalHeaderLabels([str(string) for string in self.xAxis])
#        self.tableWidgetIgnition.setVerticalHeaderLabels([str(string) for string in reversed(self.yAxis)])
#        self.tableWidgetIgnition_2.setHorizontalHeaderLabels([str(string) for string in self.xAxis])
#        self.tableWidgetIgnition_2.setVerticalHeaderLabels([str(string) for string in reversed(self.yAxis)])
#        self.tableWidgetIgnition_3.setHorizontalHeaderLabels([str(string) for string in self.xAxis])
#        self.tableWidgetIgnition_3.setVerticalHeaderLabels([str(string) for string in reversed(self.yAxis)])
##        self.tableWidgetIgnition.item(3, 5).setBackground(QColor(100,100,150))
              
 
    def PopulateSerialComboBox(self):
        AvailableSerialPort = self.serial_ports()
        self.comboBoxSerialPort.clear()
        for Port in AvailableSerialPort:
            self.comboBoxSerialPort.addItem(Port)

    def ConnectWithMechaduino(self, Port):          
   
        try:
            SerialPort = str(self.comboBoxSerialPort.currentText())
            self.MechaduinoCommunication = MechaduinoCommunication(SerialPort, 115200)        
            
        except:
            #TODO emit error code connection failed need to be loaded.                
            self.textBrowserComStatus.append("Connection to serialport " + str(SerialPort) + " failed")  
            self.MechaduinoCommunication = None
                
        if self.MechaduinoCommunication:
#            self.IonLogger =  IonSenseDatalogger(self.xAxis, self.yAxis, self.zValues) 
            self.textBrowserComStatus.append("Connected to mechaduino on serialport: " + str(SerialPort)) 
            QApplication.instance().lastWindowClosed.connect(self.MechaduinoCommunication.stop)
            self.MechaduinoCommunication.newData.connect(self.UpdateWithNewData)
            self.MechaduinoCommunication.start()
            self.ConnectMechaduino.clicked.disconnect(self.ConnectWithMechaduino)
            self.ConnectMechaduino.clicked.connect(self.DisconnectWithMechaduino)
            self.ConnectMechaduino.setText("Disconnect")
            
            self.do_pStep.setEnabled(True)
            self.do_vStep.setEnabled(True)
            self.do_tStep.setEnabled(True)
            
            self.setGainsvPID.setEnabled(True)
            self.setGainspPID.setEnabled(True)

    def DisconnectWithMechaduino(self):
        if self.MechaduinoCommunication:
            self.textBrowserComStatus.append("Disconnected from mechaduino")
            self.MechaduinoCommunication.stop()
            
        self.ConnectMechaduino.clicked.disconnect(self.DisconnectWithMechaduino)
        self.ConnectMechaduino.clicked.connect(self.ConnectWithMechaduino)
        self.ConnectMechaduino.setText("Connect")
        
        self.do_pStep.setEnabled(False)
        self.do_vStep.setEnabled(False)
        self.do_tStep.setEnabled(False)
        
        self.setGainsvPID.setEnabled(False)
        self.setGainspPID.setEnabled(False)

#        print self.IonLogger.Data[6][0][0].TimeStamp

    def UpdateWithNewData(self):
        if self.MechaduinoCommunication:
            DataReceived = self.MechaduinoCommunication.getData()
#            rpmBin, mapBin, nrOfLoggedItemsInBin = self.IonLogger.AddItem(DataReceived)
            
            #find selected row and colum in table
#            row = self.tableWidgetIgnition_2.currentRow()
#            column = self.tableWidgetIgnition_2.currentColumn()
                               
            #set background color of current measured bin and number of logged items in bin                              
#            G = int(np.clip(nrOfLoggedItemsInBin*25.5, 0, 255))
#            R = 255-G
#            B = 0
#            rpmlen = len(self.rpmAxis) - 1
#            maplen = len(self.mapAxis) - 1
#            self.tableWidgetIgnition_2.setItem(maplen-mapBin , rpmBin, QTableWidgetItem(str(nrOfLoggedItemsInBin)))
#            self.tableWidgetIgnition_2.item(maplen-mapBin , rpmBin).setBackground(QColor(R,G,B))   
            self.UpdateLivePlot(DataReceived)
            self.textBrowserStepInfo.clear()
            self.textBrowserStepInfo.append("Fs:" +str(DataReceived.Fs) + "\t mode:" + str(DataReceived.mode))
            self.textBrowserStepInfo.append("pPGain:" +str(DataReceived.pPGain) + "\t pIGain:" +str(DataReceived.pIGain) + "\t pDGain:" +str(DataReceived.pDGain))
            self.textBrowserStepInfo.append("vPGain:" +str(DataReceived.vPGain) + "\t vIGain:" +str(DataReceived.vIGain) + "\t vDGain:" +str(DataReceived.vDGain))
            
#            self.lcdNumberMap.display(DataReceived.map)
#            self.lcdNumberRPM.display(DataReceived.rpm)

    def serial_ports(self):
        """ Lists serial port names
    
            :raises EnvironmentError:
                On unsupported or unknown platforms
                
            :returns:
                A list of the serial ports available on the system
        """
        if sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this excludes your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')
            ports = ports + glob.glob('/dev/rfcomm*')
            
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')
    
        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result


        
class SubDialog(QDialog):
    def setupUi(self, Dialog):
        Dialog.resize(532, 285)

app = QtGui.QApplication(sys.argv)
window = Ui_MechaduinoMainWindow()
window.show()

Table = pg.TableWidget()


sys.exit(app.exec_())

   