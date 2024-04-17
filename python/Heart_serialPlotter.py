#!/usr/bin/env python3
# Plots the output from a single MAX3010x heart sensor being sent from
# a SensorChecker board (ATmega4808), running the Python_output_RevA.ino sketch
# Run this in a recent Python3 environment
# 
# Tested working with SensorChecker Rev A and B on Mac
#  This script handles 4-byte (float) values currently
# On a Mac terminal, try running: ls /dev/tty.usb* to find the connected usb FTDI adapter names
# and then enter the appropriate one in the portName variable down below in def main():
#
# To run this program in a Mac terminal, navigate to the directory where this file sits, and
# the at the terminal run: python3 Heart_serialPlotter.py
 
from threading import Thread
import serial
import time
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import struct
import pandas as pd
 
 
class serialPlot:
    def __init__(self, serialPort = '/dev/ttyUSB0', serialBaud = 38400, plotLength = 100, dataNumBytes = 2):
        self.port = serialPort
        self.baud = serialBaud
        self.plotMaxLength = plotLength
        self.dataNumBytes = dataNumBytes
        self.rawData = bytearray(dataNumBytes)
        self.data = collections.deque([0] * plotLength, maxlen=plotLength)
        self.isRun = True
        self.isReceiving = False
        self.thread = None
        self.plotTimer = 0
        self.previousTimer = 0
        # self.csvData = []
 
        print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
            print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
 
    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)
 
    def getSerialData(self, frame, lines, lineValueText, lineLabel, timeText):
        currentTimer = time.perf_counter()
        self.plotTimer = int((currentTimer - self.previousTimer) * 1000)     # the first reading will be erroneous
        self.previousTimer = currentTimer
        timeText.set_text('Plot Interval = ' + str(self.plotTimer) + 'ms')
        value,  = struct.unpack('f', self.rawData)    # use 'h' for a 2 byte integer
        self.data.append(value)    # we get the latest data point and append it to our array
        lines.set_data(range(self.plotMaxLength), self.data)
        lineValueText.set_text('[' + lineLabel + '] = ' + str(value))
        # self.csvData.append(self.data[-1])
 
    def backgroundThread(self):    # retrieve data
        time.sleep(1.0)  # give some buffer time for retrieving data
        self.serialConnection.reset_input_buffer()
        while (self.isRun):
            self.serialConnection.readinto(self.rawData)
            self.isReceiving = True
            #print(self.rawData)
 
    def close(self):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')
        # df = pd.DataFrame(self.csvData)
        # df.to_csv('/home/rikisenia/Desktop/data.csv')
 
 
def main():
    # On a mac, try ls /dev/tty.usb* to find attached FTDI adapters
    # portName = 'COM6'     # for windows users
    #portName = '/dev/tty.usbmodemFA131'  # Uno on the mac laptop
#    portName = '/dev/tty.usbmodem39523001' # Teensy3.5 on the Lurida Mac laptop
    portName = '/dev/tty.usbserial-A104JYLR' # SensorChecker on Lurida Mac laptop
    # portName = '/dev/tty.usbmodem82462201' # Teensy3.5 on the mac laptop 
    baudRate = 57600
    maxPlotLength = 200
    dataNumBytes = 4        # number of bytes of 1 data point
    s = serialPlot(portName, baudRate, maxPlotLength, dataNumBytes)   # initializes all required variables
    s.readSerialStart()                                               # starts background thread
 
    # plotting starts below
    pltInterval = 50    # Period at which the plot animation updates [ms]
    xmin = 0
    xmax = maxPlotLength
    ymin = 100000  # Consider raising this to a larger non-zero value to help accentuate the signal
    ymax = 120000
    fig = plt.figure()
    ax = plt.axes(xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    ax.set_title('Pulse sensor output')
    ax.set_xlabel("time")
    ax.set_ylabel("Raw Value")
 
    lineLabel = 'Serial Value'
    timeText = ax.text(0.50, 0.95, '', transform=ax.transAxes)
    lines = ax.plot([], [], label=lineLabel)[0]
    lineValueText = ax.text(0.50, 0.90, '', transform=ax.transAxes)
    anim = animation.FuncAnimation(fig, s.getSerialData, fargs=(lines, lineValueText, lineLabel, timeText), interval=pltInterval)    # fargs has to be a tuple
 
    plt.legend(loc="upper left")
    plt.show()
 
    s.close()
 
 
if __name__ == '__main__':
    main()
