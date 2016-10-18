"""
ldr.py
Display analog data from Arduino using Python (matplotlib)
Author: Mahesh Venkitachalam
Website: electronut.in
"""

import sys, serial, argparse
import numpy as np
from time import sleep
from collections import deque

import matplotlib.pyplot as plt 
import matplotlib.animation as animation

    
# plot class
class AnalogPlot:
  # constr
  def __init__(self, strPort, maxLen):
      # open serial port
      self.ser = serial.Serial(strPort, 115200)

      self.ax0 = deque([0.0]*maxLen)
      self.ax1 = deque([0.0]*maxLen)
      self.ax2 = deque([0.0]*maxLen)
      self.ax3 = deque([0.0]*maxLen)
      self.ax4 = deque([0.0]*maxLen)
      self.ax5 = deque([0.0]*maxLen)
      self.ax6 = deque([0.0]*maxLen)
      self.ax7 = deque([0.0]*maxLen)
      self.ax8 = deque([0.0]*maxLen)
      
      self.maxLen = maxLen

  # add to buffer
  def addToBuf(self, buf, val):
      if len(buf) < self.maxLen:
          buf.append(val)
      else:
          buf.pop()
          buf.appendleft(val)

  # add data
  def add(self, data):
      assert(len(data) == 9)
      self.addToBuf(self.ax0, data[0])
      self.addToBuf(self.ax1, data[1])
      self.addToBuf(self.ax2, data[2])
      self.addToBuf(self.ax3, data[4])
      self.addToBuf(self.ax4, data[4])
      self.addToBuf(self.ax5, data[5])
      self.addToBuf(self.ax6, data[6])
      self.addToBuf(self.ax7, data[7])
      self.addToBuf(self.ax8, data[8])
      

  # update plot
  def update(self, frameNum, a0, a1, a2, a3, a4, a5, a6, a7, a8):
      try:
          line = self.ser.readline()
          data = [float(val) for val in line.split()]
          temp = 'cat'
          # print data
          
          if(len(data) == 9):
              self.add(data)
              a0.set_data(range(self.maxLen), self.ax0)
              a1.set_data(range(self.maxLen), self.ax1)
              a2.set_data(range(self.maxLen), self.ax2)
              a3.set_data(range(self.maxLen), self.ax3)
              a4.set_data(range(self.maxLen), self.ax4)
              a5.set_data(range(self.maxLen), self.ax5)
              a6.set_data(range(self.maxLen), self.ax6)
              a7.set_data(range(self.maxLen), self.ax7)
              a8.set_data(range(self.maxLen), self.ax8)

          self.ser.write(temp) 
          
      except ValueError:
          print('Not a float')
      
      return a0, 

  # clean up
  def close(self):
      # close serial
      self.ser.flush()
      self.ser.close()    

# main() function
def main():
  # create parser
  parser = argparse.ArgumentParser(description="LDR serial")
  # add expected arguments
  parser.add_argument('--port', dest='port', required=True)

  # parse args
  args = parser.parse_args()
  
  #strPort = '/dev/tty.usbserial-A7006Yqh'
  
  #strPort = 'COM8'
  strPort = args.port

  print('reading from serial port %s...' % strPort)

  # plot parameters
  analogPlot = AnalogPlot(strPort, 100)

  print('plotting data...')

  # set up animation
  #plt.subplot(223)
  fig = plt.figure()
  ax = plt.axes(xlim=(0, 100), ylim=(-1023, 1023))
  a0, = ax.plot([], label="accX")
  a1, = ax.plot([], label="accY")
  a2, = ax.plot([], label="accZ")
  a3, = ax.plot([], label="gryoX")
  a4, = ax.plot([], label="gryoY")
  a5, = ax.plot([], label="gryoZ")
  a6, = ax.plot([], label="magX")
  a7, = ax.plot([], label="magY")
  a8, = ax.plot([], label="magHeading")
  
  anim = animation.FuncAnimation(fig, analogPlot.update, 
                                 fargs=(a0, a1, a2, a3, a4, a5, a6, a7, a8), 
                                 interval=50)



  plt.legend()
  #bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)

  # show plot
  plt.show()
  
  # clean up
  analogPlot.close()

  print('exiting.')
  

# call main
if __name__ == '__main__':
  main()
