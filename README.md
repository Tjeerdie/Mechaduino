# Mechaduino  

Mechaduino firmware and GUI  
This repository holds my python GUI to tune the PID of the mechaduino. The firmware is a simple adaptation of the original firmware of Tropical Labs to interact with the gains of the PID's.  

Mechaduino:  
upload the firmware to the mechaduino with the arduino IDE. See the Tropical labs manual for more information  

GUI:  
The GUI is written in python 2.7, you need a couple of additional libraries in python at least:  
- Pyqt4  
- Pyqtgraph  
install with pip or your favorite python package manager.   

Usage: 
First connect to your mechaduino in the "communication settings" tab. Select the serial port where the mechaduino is connected to and click connect.  

Go to the step response tab and put the desired gains for the PID controllers. Please note that the D term of the speed controller from tropical labs is incorrect (substraced instead of added), therefore the default D gain for the speed controller is negative (that should not be). the buttons "set v PID" and "set p PID" send the gains to the mechaduino. These gains will not remain in the mechaduino after a power cycle. If you want to make the gains permanent please adjust them accordingly in the mechaduino firmware.  

Use the vStep and pStep buttons to create the desired step inputs too see the response of the controller. tune the gains as desired and test with the step response. If you are satisfied with the results make them permanent by changing the gains in the mechaduino firmware. 

 
