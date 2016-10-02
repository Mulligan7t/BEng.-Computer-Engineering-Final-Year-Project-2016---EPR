#!/usr/bin/python
from __future__ import division
from sys import argv
import zbar
from PIL import Image
from graphics import *
if len(argv) < 2: exit(1)
import sys, serial, argparse
import numpy as np
from time import sleep
from collections import deque

import socket


def drive(coX0, coX1, coY0, coY1):
  LFspeed = 100 
  RFspeed = 100
  LBspeed = 100
  RBspeed = 100

  coXdiff = coX0-coX1
  coYdiff = coY0-coY1
  


  return str(LFspeed) + " " + str(RFspeed) + " " + str(LBspeed) + " " + str(RBspeed) + "\r"
 
# main() function
def main():
  if len(argv) < 2: exit(1)

  # create a reader
  scanner = zbar.ImageScanner()

  # configure the reader
  scanner.parse_config('enable')

  # obtain image data
  pil = Image.open(argv[1]).convert('L')
  width, height = pil.size
  raw = pil.tostring()

  # wrap image data
  image = zbar.Image(width, height, 'Y800', raw)

  # scan the image for barcodes
  scanner.scan(image)

  # extract results
  for symbol in image:
      # do something useful with results
      print 'decoded', symbol.type, symbol.location, 'symbol', '"%s"' % symbol.data
      barloc = symbol.location


  print barloc[0][0]

  # clean up
  del(image)

#  Address=("127.0.0.1",5000)
#  s = socket.socket()
#  try:
#    s.connect(Address)
#  except Exception, e:
  print "The server is not running"
      

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

  A = 0
  B = 1
  C = 2
  D = 3
  E = 4
  F = 5
  G = 6

  # A X - -
  # - X - -
  # - X X X
  # - - - -
  
  route  = (A,0),(A,1),(B,1),(C,1),(C,2),(C,3)
  routelen = 5

  print route[0][0]
  print route[1]



  print('reading from serial port %s...' % strPort)

  print('plotting data...')

  
  frameCnt = 0
  while True:
    try:
      line = ser.readline()
      data = [float(val) for val in line.split()]
     
      # print data
      frameCnt = int(cnt / 10)

      if frameCnt < routelen:
        ser.write(drive(route[frameCnt][0],route[frameCnt+1][0],
                    route[frameCnt][1],route[frameCnt+1][1])) 
        #ser.write(driveY() 

      
      if(len(data) == 10):
        dt = data[9]
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

      
  
  
    except ValueError:
        print('Not a float')
        
    except KeyboardInterrupt:
        ser.write("0 0 0 0\r")
        raise SystemExit

      

  
  print('exiting.')
  

# call main
if __name__ == '__main__':
  main()
