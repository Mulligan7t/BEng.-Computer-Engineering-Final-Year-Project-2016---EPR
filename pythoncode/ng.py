import sys, serial, argparse
import numpy as np
from time import sleep
from collections import deque

import matplotlib.pyplot as plt 
import matplotlib.animation as animation

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

  ser = serial.Serial(strPort, 115200)


  cnt = 1
  velX = 0
  posX = 0
  totvelX = 0
  totposX = 0
  velY = 0
  calib = 0
  totcalib = 0
  calibcnt = 100
  dt = .05
  print('reading from serial port %s...' % strPort)

  print('plotting data...')

  
  frameCnt = 0
  while True:
    try:
      line = ser.readline()
      data = [float(val) for val in line.split()]
      temp = 'cat'
      # print data
      frameCnt += 1
      
      if(len(data) == 10):
        
        accXcurr = data[0]- calib
        totvelX += accXcurr*dt
        if (cnt < calibcnt):
          totcalib += accXcurr
          calib = totcalib/cnt
        #velX = totvelX/cnt
        velX += accXcurr*dt
        totposX += velX*dt
        #posX = totposX/cnt
        posX += velX*dt
        velY += data[1]/cnt 
        print('X\'\' %s' % accXcurr)
        print('X\'  %s' % velX)
        print('X   %s' % posX)
        print(' ')
        cnt += 1

      ser.write(temp) 
    
    except ValueError:
        print('Not a float')
      

  
  print('exiting.')
  

# call main
if __name__ == '__main__':
  main()
