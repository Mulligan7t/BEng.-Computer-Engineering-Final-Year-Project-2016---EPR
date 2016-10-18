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
import RPi.GPIO as GPIO
import threading

                          # GPIO Ports
Enc_0_A = 21              # Encoder input A: input GPIO 21
Enc_0_B = 20              # Encoder input B: input GPIO 20
Enc_1_A = 1               # Encoder input A: input GPIO 1 
Enc_1_B = 0               # Encoder input B: input GPIO 0
Enc_2_A = 2               # Encoder input A: input GPIO 2 
Enc_2_B = 3               # Encoder input B: input GPIO 3
Enc_3_A = 14              # Encoder input A: input GPIO 14 
Enc_3_B = 15              # Encoder input B: input GPIO 15




Rotary_counter_0 = 0                # Start counting from 0
Current_0_A = 1                     # Assume that rotary switch is not 
Current_0_B = 1                     # moving while we init software
LockRotary_0 = threading.Lock()     # create lock for rotary switch


Rotary_counter_1 = 0                # Start counting from 0
Current_1_A = 1                     # Assume that rotary switch is not 
Current_1_B = 1                     # moving while we init software
LockRotary_1 = threading.Lock()     # create lock for rotary switch
   
Rotary_counter_2 = 0                # Start counting from 0
Current_2_A = 1                     # Assume that rotary switch is not 
Current_2_B = 1                     # moving while we init software
LockRotary_2 = threading.Lock()     # create lock for rotary switch

Rotary_counter_3 = 0                # Start counting from 0
Current_3_A = 1                     # Assume that rotary switch is not 
Current_3_B = 1                     # moving while we init software
LockRotary_3 = threading.Lock()     # create lock for rotary switch


WHEEL_RADIUS=30
WHEEL_SEPARATION_WIDTH = 93
WHEEL_SEPARATION_LENGTH = 90
linearX = 000
linearY = 2000
angularZ = 0
speedcalib = 2.55

LFspeed = 0
RFspeed = 0
LBspeed = 0
RBspeed = 0

LFset = 0
RFset = 0
LBset = 0
RBset = 0

def drive(coX0, coX1, coY0, coY1):
  global LFset, RFset, LBset, RBset

  LFset = (1/WHEEL_RADIUS) * (linearX - linearY - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularZ) * speedcalib
  RFset = (1/WHEEL_RADIUS) * (linearX + linearY + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularZ) * speedcalib
  LBset = (1/WHEEL_RADIUS) * (linearX + linearY - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularZ) * speedcalib
  RBset = (1/WHEEL_RADIUS) * (linearX - linearY + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularZ) * speedcalib

  coXdiff = coX0-coX1
  coYdiff = coY0-coY1
    
  LFset = 255 
  RFset = 0
  LBset = 0
  RBset = 0

  LFset = LFset//7
  RFset = RFset//7
  LBset = LBset//7
  RBset = RBset//7



  print "SET:   " + str(LFset) + " " + str(RFset) + " " + str(LBset) + " " + str(RBset) + "\r"

def encoderfeedback():
  global LFspeed, RFspeed, LBspeed, RBspeed
  LFspeed = int(LFspeed + math.ceil(0.1*(LFset-speed_0)))
  RFspeed = int(RFspeed + math.ceil(0.1*(RFset-speed_1)))
  LBspeed = int(LBspeed + math.ceil(0.1*(LBset-speed_2)))
  RBspeed = int(RBspeed + math.ceil(0.1*(RBset-speed_3)))

  if(LFspeed>254):
    LFspeed = 254
  if(RFspeed>254):
    RFspeed = 254
  if(LBspeed>254):
    LBspeed = 254
  if(RBspeed>254):
    RBspeed = 254


  print "PWM:   " + str(LFspeed) + " " + str(RFspeed) + " " + str(LBspeed) + " " + str(RBspeed) + "\r"
  return str(LFspeed) + " " + str(RFspeed) + " " + str(LBspeed) + " " + str(RBspeed) + "\r"


def camqr():
  if len(argv) < 2: exit(1)

  # create a reader
  scanner = zbar.ImageScanner()

  # configure the reader
  scanner.parse_config('enable')

  # obtain image data
  pil = Image.open('image.jpg').convert('L')
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

  win = GraphWin('qr', 500, 500)
  pt = Circle(Point(barloc[0][0],barloc[0][1]),20)
  pt.draw(win)
  pt = Circle(Point(barloc[1][0],barloc[1][1]),5)
  pt.draw(win) 
  i = 0
  while (i<3):
      line = Line(Point(barloc[i][0],barloc[i][1]), Point(barloc[i+1][0], barloc[i+1][1]))
      line.draw(win)
      i = i + 1
  line = Line(Point(barloc[i][0],barloc[i][1]), Point(barloc[0][0], barloc[0][1]))
  line.draw(win)
  sqrt =((barloc[0][0] - barloc[2][0])**2 + (barloc[0][1] - barloc[2][1])**2)**0.5
  a =float(barloc[0][0] - barloc[2][0])
  b =float(barloc[0][1] - barloc[2][1])
  a = a / sqrt
  b = b / sqrt
  print sqrt
  print a
  print b

  a = a * 100
  b = b * 100
  c = 250

  line = Line(Point(c,c), Point(c+a,c+b))
  line.draw(win)

  #win.getMouse() #pause for click in window

# initialize interrupt handlers
def init():
   GPIO.setwarnings(True)
   GPIO.setmode(GPIO.BCM)               # Use BCM mode
                                 # define the Encoder switch inputs
   GPIO.setup(Enc_0_A, GPIO.IN)             
   GPIO.setup(Enc_0_B, GPIO.IN)
   GPIO.setup(Enc_1_A, GPIO.IN)             
   GPIO.setup(Enc_1_B, GPIO.IN)
   GPIO.setup(Enc_2_A, GPIO.IN)             
   GPIO.setup(Enc_2_B, GPIO.IN)
   GPIO.setup(Enc_3_A, GPIO.IN)             
   GPIO.setup(Enc_3_B, GPIO.IN)


                                 # setup callback thread for the A and B encoder 
                                 # use interrupts for all inputs
   GPIO.add_event_detect(Enc_0_A, GPIO.RISING, callback=rotary_interrupt_0)             # NO bouncetime 
   GPIO.add_event_detect(Enc_0_B, GPIO.RISING, callback=rotary_interrupt_0)             # NO bouncetime 
   GPIO.add_event_detect(Enc_1_A, GPIO.RISING, callback=rotary_interrupt_1)             # NO bouncetime 
   GPIO.add_event_detect(Enc_1_B, GPIO.RISING, callback=rotary_interrupt_1)             # NO bouncetime 
   GPIO.add_event_detect(Enc_2_A, GPIO.RISING, callback=rotary_interrupt_2)             # NO bouncetime 
   GPIO.add_event_detect(Enc_2_B, GPIO.RISING, callback=rotary_interrupt_2)             # NO bouncetime 
   GPIO.add_event_detect(Enc_3_A, GPIO.RISING, callback=rotary_interrupt_3)             # NO bouncetime 
   GPIO.add_event_detect(Enc_3_B, GPIO.RISING, callback=rotary_interrupt_3)             # NO bouncetime 

   return



# Rotarty encoder interrupt:
# this one is called for both inputs from rotary switch (A and B)
def rotary_interrupt_0(A_or_B):
   global Rotary_counter_0, Current_0_A, Current_0_B, LockRotary_0
                                       # read both of the switches
   Switch_A = GPIO.input(Enc_0_A)
   Switch_B = GPIO.input(Enc_0_B)
                                       # now check if state of A or B has changed
                                       # if not that means that bouncing caused it
   if Current_0_A == Switch_A and Current_0_B == Switch_B:      # Same interrupt as before (Bouncing)?
      return                                                    # ignore interrupt!

   Current_0_A = Switch_A                        # remember new state
   Current_0_B = Switch_B                        # for next bouncing check


   if (Switch_A and Switch_B):                  # Both one active? Yes -> end of sequence
      LockRotary_0.acquire()                    # get lock 
      if A_or_B == Enc_0_B:                     # Turning direction depends on 
         Rotary_counter_0 += 1                  # which input gave last interrupt
      else:                                     # so depending on direction either
         Rotary_counter_0 -= 1                  # increase or decrease counter
      LockRotary_0.release()                    # and release lock
   return                                       # THAT'S IT

# Rotarty encoder interrupt:
# this one is called for both inputs from rotary switch (A and B)
def rotary_interrupt_1(A_or_B):
   global Rotary_counter_1, Current_1_A, Current_1_B, LockRotary_1
                                       # read both of the switches
   Switch_A = GPIO.input(Enc_1_A)
   Switch_B = GPIO.input(Enc_1_B)
                                       # now check if state of A or B has changed
                                       # if not that means that bouncing caused it
   if Current_1_A == Switch_A and Current_1_B == Switch_B:      # Same interrupt as before (Bouncing)?
      return                                                    # ignore interrupt!

   Current_1_A = Switch_A                        # remember new state
   Current_1_B = Switch_B                        # for next bouncing check


   if (Switch_A and Switch_B):                  # Both one active? Yes -> end of sequence
      LockRotary_1.acquire()                    # get lock 
      if A_or_B == Enc_1_B:                     # Turning direction depends on 
         Rotary_counter_1 += 1                  # which input gave last interrupt
      else:                                     # so depending on direction either
         Rotary_counter_1 -= 1                  # increase or decrease counter
      LockRotary_1.release()                    # and release lock
   return                                       # THAT'S IT

# Rotarty encoder interrupt:
# this one is called for both inputs from rotary switch (A and B)
def rotary_interrupt_2(A_or_B):
   global Rotary_counter_2, Current_2_A, Current_2_B, LockRotary_2
                                       # read both of the switches
   Switch_A = GPIO.input(Enc_2_A)
   Switch_B = GPIO.input(Enc_2_B)
                                       # now check if state of A or B has changed
                                       # if not that means that bouncing caused it
   if Current_2_A == Switch_A and Current_2_B == Switch_B:      # Same interrupt as before (Bouncing)?
      return                                                    # ignore interrupt!

   Current_2_A = Switch_A                        # remember new state
   Current_2_B = Switch_B                        # for next bouncing check


   if (Switch_A and Switch_B):                  # Both one active? Yes -> end of sequence
      LockRotary_2.acquire()                    # get lock 
      if A_or_B == Enc_2_B:                     # Turning direction depends on 
         Rotary_counter_2 += 1                  # which input gave last interrupt
      else:                                     # so depending on direction either
         Rotary_counter_2 -= 1                  # increase or decrease counter
      LockRotary_2.release()                    # and release lock
   return                                       # THAT'S IT

# Rotarty encoder interrupt:
# this one is called for both inputs from rotary switch (A and B)
def rotary_interrupt_3(A_or_B):
   global Rotary_counter_3, Current_3_A, Current_3_B, LockRotary_3
                                       # read both of the switches
   Switch_A = GPIO.input(Enc_3_A)
   Switch_B = GPIO.input(Enc_3_B)
                                       # now check if state of A or B has changed
                                       # if not that means that bouncing caused it
   if Current_3_A == Switch_A and Current_3_B == Switch_B:      # Same interrupt as before (Bouncing)?
      return                                                    # ignore interrupt!

   Current_3_A = Switch_A                        # remember new state
   Current_3_B = Switch_B                        # for next bouncing check


   if (Switch_A and Switch_B):                  # Both one active? Yes -> end of sequence
      LockRotary_3.acquire()                    # get lock 
      if A_or_B == Enc_3_B:                     # Turning direction depends on 
         Rotary_counter_3 += 1                  # which input gave last interrupt
      else:                                     # so depending on direction either
         Rotary_counter_3 -= 1                  # increase or decrease counter
      LockRotary_3.release()                    # and release lock
   return                                       # THAT'S IT
 

# main() function
def main():
  
  #  camqr()
  #  Address=("127.0.0.1",5000)
  #  s = socket.socket()
  #  try:
  #    s.connect(Address)
  #  except Exception, e:

  print "The server is not running"
  
  global Rotary_counter_0, LockRotary_0, Rotary_counter_1, LockRotary_1,Rotary_counter_2, LockRotary_2, Rotary_counter_3, LockRotary_3, speed_0, speed_1, speed_2, speed_3

  TotalCount_0 = 0                            # Current TotalCount_0   
  NewCounter_0 = 0                            # for faster reading with locks           
  speed_0 = 0
  TotalCount_1 = 0                            # Current TotalCount_1   
  NewCounter_1 = 0                            # for faster reading with locks           
  speed_1 = 0
  TotalCount_2 = 05                           # Current TotalCount_2   
  NewCounter_2 = 0                            # for faster reading with locks           
  speed_2 = 0
  TotalCount_3 = 0                            # Current TotalCount_3   
  NewCounter_3 = 0                            # for faster reading with locks           
  speed_3 = 0
  cnt = 0 

  init()                              # Init interrupts, GPIO, ...


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
  printx = 0
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
        #ser.write()
        drive(route[frameCnt][0],route[frameCnt+1][0], route[frameCnt][1],route[frameCnt+1][1])
        ser.write(encoderfeedback())
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
        if printx:
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

#    sleep(0.1)                        # sleep 100 msec
    cnt = cnt +1
                                  # because of threading make sure no thread
                                  # changes value until we get them
                                  # and reset them
                                  
    LockRotary_0.acquire()               # get lock for rotary switch
    NewCounter_0 = Rotary_counter_0      # get counter value
    Rotary_counter_0 = 0                 # RESET IT TO 0
    LockRotary_0.release()               # and release lock
             
    if (NewCounter_0 !=0):               # Counter has CHANGED
       TotalCount_0 = TotalCount_0 + NewCounter_0

    LockRotary_1.acquire()               # get lock for rotary switch
    NewCounter_1 = Rotary_counter_1      # get counter value
    Rotary_counter_1 = 0                 # RESET IT TO 0
    LockRotary_1.release()               # and release lock
             
    if (NewCounter_1 !=0):               # Counter has CHANGED
       TotalCount_1 = TotalCount_1 + NewCounter_1

    LockRotary_2.acquire()               # get lock for rotary switch
    NewCounter_2 = Rotary_counter_2      # get counter value
    Rotary_counter_2 = 0                 # RESET IT TO 0
    LockRotary_2.release()               # and release lock
             
    if (NewCounter_2 !=0):               # Counter has CHANGED
       TotalCount_2 = TotalCount_2 + NewCounter_2

    LockRotary_3.acquire()               # get lock for rotary switch
    NewCounter_3 = Rotary_counter_3      # get counter value
    Rotary_counter_3 = 0                 # RESET IT TO 0
    LockRotary_3.release()               # and release lock
             
    if (NewCounter_3 !=0):               # Counter has CHANGED
       TotalCount_3 = TotalCount_3 + NewCounter_3

                               
    if (cnt > 10):
      speed_0 = (TotalCount_0/cnt)
      speed_1 = (TotalCount_1/cnt)
      speed_2 = (TotalCount_2/cnt)
      speed_3 = (TotalCount_3/cnt)
      print speed_0, speed_1, speed_2, speed_3
      cnt = 0
      TotalCount_0 = 0
      TotalCount_1 = 0
      TotalCount_2 = 0
      TotalCount_3 = 0

      

  
  print('exiting.')
  

# call main
if __name__ == '__main__':
  main()
