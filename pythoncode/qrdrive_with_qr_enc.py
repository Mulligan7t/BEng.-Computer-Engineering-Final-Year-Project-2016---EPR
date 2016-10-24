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
import math

                          # GPIO Ports
Enc_1_A = 21              # Encoder input A: input GPIO 21
Enc_1_B = 20              # Encoder input B: input GPIO 20
Enc_0_A = 1               # Encoder input A: input GPIO 1 
Enc_0_B = 0               # Encoder input B: input GPIO 0
Enc_2_A = 2               # Encoder input A: input GPIO 2 
Enc_2_B = 3               # Encoder input B: input GPIO 3
Enc_3_A = 14              # Encoder input A: input GPIO 14 
Enc_3_B = 15              # Encoder input B: input GPIO 15



rotary_counters = [0,0,0,0]
lock_rotary = [threading.Lock(),threading.Lock(),threading.Lock(),threading.Lock()]     # create lock for rotary switch


Current_0_A = 1                     # Assume that rotary switch is not 
Current_0_B = 1                     # moving while we init software

Current_1_A = 1                     # Assume that rotary switch is not 
Current_1_B = 1                     # moving while we init software
   
Current_2_A = 1                     # Assume that rotary switch is not 
Current_2_B = 1                     # moving while we init software

Current_3_A = 1                     # Assume that rotary switch is not 
Current_3_B = 1                     # moving while we init software


WHEEL_RADIUS=30
WHEEL_SEPARATION_WIDTH = 93
WHEEL_SEPARATION_LENGTH = 90
linearX = -2000                       #Forward (+ to the front)
linearY = 000                      #Sideways (+ to the left)
angularZ = 0 
speedcalib = 2.55

PWMoutput = [0,0,0,0]

motor_setpoint = [0,0,0,0]

Kp = 0.5
Ki = 0.2
Kd = 0.1

LFintegral = 0
LFprev_error = 0

min_interia = 254 #minimum PWM to break intertia and start turning
min_dynamic = 150 #lower than this and will stall even after rotation has begun

left_front = 0
right_front = 1
left_back = 2
right_back = 3

def drive(coX0, coX1, coY0, coY1):
  global motor_setpoint
 

  motor_setpoint[left_front] = (1/WHEEL_RADIUS) * (linearX - linearY - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularZ) * speedcalib
  motor_setpoint[right_front] = (1/WHEEL_RADIUS) * (linearX + linearY + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularZ) * speedcalib
  motor_setpoint[left_back] = (1/WHEEL_RADIUS) * (linearX + linearY - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularZ) * speedcalib
  motor_setpoint[right_back] = (1/WHEEL_RADIUS) * (linearX - linearY + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularZ) * speedcalib

  coXdiff = coX0-coX1
  coYdiff = coY0-coY1
  
  if (False):
    motor_setpoint[left_front] = 255 
    motor_setpoint[right_front] = 255 
    motor_setpoint[left_back] = 255
    motor_setpoint[right_back] = 255

  encoderRate = 7
  motor_setpoint[left_front] = motor_setpoint[left_front]//encoderRate
  motor_setpoint[right_front] = motor_setpoint[right_front]//encoderRate
  motor_setpoint[left_back] = motor_setpoint[left_back]//encoderRate
  motor_setpoint[right_back] = motor_setpoint[right_back]//encoderRate



  print "SET:   " + str(motor_setpoint[left_front]) + " " + str(motor_setpoint[right_front]) + " " + str(motor_setpoint[left_back]) + " " + str(motor_setpoint[right_back]) + "\r"

def encoderfeedback():
  global PWMoutput, LFintegral, LFprev_error
  #PWMoutput[left_front] = PWMoutput[left_front] + math.ceil(0.1*(motor_setpoint[left_front]-speed_0))
  #PWMoutput[right_front] = PWMoutput[right_front] + math.ceil(0.1*(motor_setpoint[right_front]-speed_1)) 
  #PWMoutput[left_back] = PWMoutput[left_back] + math.ceil(0.1*(motor_setpoint[left_back]-speed_2))
  #PWMoutput[right_back] = PWMoutput[right_back] + math.ceil(0.1*(motor_setpoint[right_back]-speed_3))

  #if the wheel is not turning i.e. (measured) speed_0 = 0.0, then set to min_interia

  LFerror = motor_setpoint[left_front]-speed_0
  RFerror = motor_setpoint[right_front]-speed_1
  LBerror = motor_setpoint[left_back]-speed_2
  RBerror = motor_setpoint[right_back]-speed_3

  LFintegral = LFintegral + LFerror

  if((LFerror < 0.5) or (LFintegral > 254)):
    LFintegral = 0

  LFderivative = LFerror - LFprev_error
  LFprev_error = LFerror

  PWMoutput[left_front] = PWMoutput[left_front] + (Kp*LFerror + Ki*LFintegral + Kd*LFderivative)
  print "LFerror: " + str(LFerror),
  print "  LFintegral: " + str(LFintegral),
  print "  LFderv: " + str(LFderivative)
  if(speed_0==0 and math.fabs(motor_setpoint[left_front]) > 0):
    PWMoutput[left_front] = math.copysign(min_interia, motor_setpoint[left_front]) # return value of min_interia with the sign of motor_setpoint[left_front]
    print "PWMoutput[left_front] set to min_interia-----------------"

  if(math.fabs(motor_setpoint[left_front]) > 0 and math.fabs(speed_0)>1.0 and math.fabs(PWMoutput[left_front]) <min_dynamic):
    PWMoutput[left_front] = math.copysign(min_dynamic, motor_setpoint[left_front]) # return value of min_dynamic with the sign of motor_setpoint[left_front]

  
  if(math.fabs(PWMoutput[left_front])>254):
    PWMoutput[left_front] = math.copysign(254, motor_setpoint[left_front]) # return value of 254 with the sign of motor_setpoint[left_front]
  if(math.fabs(PWMoutput[right_front])>254):
    PWMoutput[right_front] = math.copysign(254, motor_setpoint[right_front]) # return value of 254 with the sign of motor_setpoint[right_front]
  if(math.fabs(PWMoutput[left_back])>254):
    PWMoutput[left_back] = math.copysign(254, motor_setpoint[left_back]) # return value of 254 with the sign of motor_setpoint[left_back]
  if(math.fabs(PWMoutput[right_back])>254):
    PWMoutput[right_back] = math.copysign(254, motor_setpoint[right_back]) # return value of 254 with the sign of motor_setpoint[right_back]

  PWMoutput[left_front] = PWMoutput[right_front] = PWMoutput[left_back] = PWMoutput[right_back] = 254
  print "speed_0:  " + str(speed_0)
  print "PWM:   " + str(int(PWMoutput[left_front])) + " " + str(int(PWMoutput[right_front])) + " " + str(int(PWMoutput[left_back])) + " " + str(int(PWMoutput[right_back])) + "\r"
  return str(int(PWMoutput[left_front])) + " " + str(int(PWMoutput[right_front])) + " " + str(int(PWMoutput[left_back])) + " " + str(int(PWMoutput[right_back])) + "\r"

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
   global rotary_counters, Current_0_A, Current_0_B, lock_rotary
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
      lock_rotary[left_front].acquire()                    # get lock 
      if A_or_B == Enc_0_B:                     # Turning direction depends on 
         rotary_counters[left_front] += 1                  # which input gave last interrupt
      else:                                     # so depending on direction either
         rotary_counters[left_front] -= 1                  # increase or decrease counter
      lock_rotary[left_front].release()                    # and release lock
   return                                       # THAT'S IT

# Rotarty encoder interrupt:
# this one is called for both inputs from rotary switch (A and B)
def rotary_interrupt_1(A_or_B):
   global rotary_counters, Current_1_A, Current_1_B, lock_rotary
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
      lock_rotary[right_front].acquire()                    # get lock 
      if A_or_B == Enc_1_B:                     # Turning direction depends on 
         rotary_counters[right_front] += 1                  # which input gave last interrupt
      else:                                     # so depending on direction either
         rotary_counters[right_front] -= 1                  # increase or decrease counter
      lock_rotary[right_front].release()                    # and release lock
   return                                       # THAT'S IT

# Rotarty encoder interrupt:
# this one is called for both inputs from rotary switch (A and B)
def rotary_interrupt_2(A_or_B):
   global rotary_counters, Current_2_A, Current_2_B, lock_rotary
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
      lock_rotary[left_back].acquire()                    # get lock 
      if A_or_B == Enc_2_B:                     # Turning direction depends on 
         rotary_counters[left_back] += 1                  # which input gave last interrupt
      else:                                     # so depending on direction either
         rotary_counters[left_back] -= 1                  # increase or decrease counter
      lock_rotary[left_back].release()                    # and release lock
   return                                       # THAT'S IT

# Rotarty encoder interrupt:
# this one is called for both inputs from rotary switch (A and B)
def rotary_interrupt_3(A_or_B):
   global rotary_counters, Current_3_A, Current_3_B, lock_rotary
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
      lock_rotary[right_back].acquire()                    # get lock 
      if A_or_B == Enc_3_B:                     # Turning direction depends on 
         rotary_counters[right_back] += 1                  # which input gave last interrupt
      else:                                     # so depending on direction either
         rotary_counters[right_back] -= 1                  # increase or decrease counter
      lock_rotary[right_back].release()                    # and release lock
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
  
  global rotary_counters, lock_rotary, speed_0, speed_1, speed_2, speed_3

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
  cntSpeed = 0 

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
      #print "line: ",
      #print line,
      
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
    cntSpeed = cntSpeed +1
                                  # because of threading make sure no thread
                                  # changes value until we get them
                                  # and reset them
                                  
    lock_rotary[left_front].acquire()               # get lock for rotary switch
    NewCounter_0 = rotary_counters[left_front]      # get counter value
    rotary_counters[left_front] = 0                 # RESET IT TO 0
    lock_rotary[left_front].release()               # and release lock
             
    if (NewCounter_0 !=0):               # Counter has CHANGED
       TotalCount_0 = TotalCount_0 + NewCounter_0

    lock_rotary[right_front].acquire()               # get lock for rotary switch
    NewCounter_1 = rotary_counters[right_front]      # get counter value
    rotary_counters[right_front] = 0                 # RESET IT TO 0
    lock_rotary[right_front].release()               # and release lock
             
    if (NewCounter_1 !=0):               # Counter has CHANGED
       TotalCount_1 = TotalCount_1 + NewCounter_1

    lock_rotary[left_back].acquire()               # get lock for rotary switch
    NewCounter_2 = rotary_counters[left_back]      # get counter value
    rotary_counters[left_back] = 0                 # RESET IT TO 0
    lock_rotary[left_back].release()               # and release lock
             
    if (NewCounter_2 !=0):               # Counter has CHANGED
       TotalCount_2 = TotalCount_2 + NewCounter_2

    lock_rotary[right_back].acquire()               # get lock for rotary switch
    NewCounter_3 = rotary_counters[right_back]      # get counter value
    rotary_counters[right_back] = 0                 # RESET IT TO 0
    lock_rotary[right_back].release()               # and release lock
             
    if (NewCounter_3 !=0):               # Counter has CHANGED
       TotalCount_3 = TotalCount_3 + NewCounter_3

                               
    if (cntSpeed > 10):
      speed_0 = (TotalCount_0/cntSpeed)
      speed_1 = (TotalCount_1/cntSpeed)
      speed_2 = (TotalCount_2/cntSpeed)
      speed_3 = (TotalCount_3/cntSpeed)
      print speed_0, speed_1, speed_2, speed_3
      cntSpeed = 0
      TotalCount_0 = 0
      TotalCount_1 = 0
      TotalCount_2 = 0
      TotalCount_3 = 0

      

  
  print('exiting.')
  

# call main
if __name__ == '__main__':
  main()
