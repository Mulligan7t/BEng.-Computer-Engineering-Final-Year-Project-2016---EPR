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

current_A = [0,0,0,0]
current_B = [0,0,0,0]

WHEEL_RADIUS=30
WHEEL_SEPARATION_WIDTH = 93
WHEEL_SEPARATION_LENGTH = 90
linearX = -2000                       #Forward (+ to the front)
linearY = 000                      #Sideways (+ to the left)
angularZ = 0 
speedcalib = 2.55

PWMoutput = [0,0,0,0]

motor_setpoint = [0,0,0,0]

encoder_reading = [0,0,0,0]

Kp = 0.5
Ki = 0.2
Kd = 0.1

integral_error = [0,0,0,0]
prev_error = [0,0,0,0]

min_interia = 254 #minimum PWM to break intertia and start turning
min_dynamic = 150 #lower than this and will stall even after rotation has begun


#array names
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

  for x_wheel in xrange(4):
    motor_setpoint[x_wheel] = motor_setpoint[x_wheel]//encoderRate

  print "SET:   " + str(motor_setpoint[left_front]) + " " + str(motor_setpoint[right_front]) + " " + str(motor_setpoint[left_back]) + " " + str(motor_setpoint[right_back]) + "\r"

def encoderfeedback():
  global PWMoutput, integral_error, prev_error
  #PWMoutput[left_front] = PWMoutput[left_front] + math.ceil(0.1*(motor_setpoint[left_front]-encoder_reading[left_front]))
  #PWMoutput[right_front] = PWMoutput[right_front] + math.ceil(0.1*(motor_setpoint[right_front]-encoder_reading[right_front])) 
  #PWMoutput[left_back] = PWMoutput[left_back] + math.ceil(0.1*(motor_setpoint[left_back]-encoder_reading[left_back]))
  #PWMoutput[right_back] = PWMoutput[right_back] + math.ceil(0.1*(motor_setpoint[right_back]-encoder_reading[right_back]))

  #if the wheel is not turning i.e. (measured) encoder_reading[left_front] = 0.0, then set to min_interia

  current_error = [0,0,0,0]
  derivative_error = [0,0,0,0]
  for x_wheel in xrange(4):
    encoder_reading[x_wheel] = motor_setpoint[x_wheel]-encoder_reading[x_wheel]

  for x_wheel in xrange(4):
    integral_error[x_wheel] = integral_error[x_wheel] + encoder_reading[x_wheel]

  for x_wheel in xrange(4):
    if((encoder_reading[x_wheel] < 0.5) or (integral_error[x_wheel] > 254)):
      integral_error[x_wheel] = 0

  for x_wheel in xrange(4):
    derivative_error[x_wheel] = encoder_reading[x_wheel] - prev_error[x_wheel]
    prev_error[x_wheel] = encoder_reading[x_wheel]
    PWMoutput[x_wheel] = PWMoutput[x_wheel] + (Kp*encoder_reading[x_wheel] + Ki*integral_error[x_wheel] + Kd*derivative_error[x_wheel])
    #print "encoder_reading[x_wheel]: " + str(encoder_reading[x_wheel]),
    #print "  integral_error[x_wheel]: " + str(integral_error[x_wheel]),
    #print "  derivative_error[x_wheel]: " + str(derivative_error[x_wheel])
  
  print "PWMoutput "
      
  for x_wheel in xrange(4):  
    if(encoder_reading[x_wheel]==0 and math.fabs(motor_setpoint[x_wheel]) > 0):
      #motor is not moving
      #motor set to run forward or backward
      PWMoutput[x_wheel] = math.copysign(min_interia, motor_setpoint[x_wheel]) # return value of min_interia with the sign of motor_setpoint[x_wheel]
      print "  ",
      print x_wheel,
      print "  ",

    if(math.fabs(motor_setpoint[x_wheel]) > 0 and math.fabs(encoder_reading[x_wheel])>1.0 and math.fabs(PWMoutput[x_wheel]) <min_dynamic):
      #motor set to run forward or backward
      #motor is running at least 1.0 forward or backward
      #PWMoutput is lower than min_dynamic
      #THEN set PWMoutput to min_dynamic
      PWMoutput[x_wheel] = math.copysign(min_dynamic, motor_setpoint[x_wheel]) # return value of min_dynamic with the sign of motor_setpoint[x_wheel]

  
    if(math.fabs(PWMoutput[x_wheel])>254):
      PWMoutput[x_wheel] = math.copysign(254, motor_setpoint[x_wheel]) # return value of 254 with the sign of motor_setpoint[x_wheel]
  
  print " set to min_interia-----------------"


  #PWMoutput[left_front] = PWMoutput[right_front] = PWMoutput[left_back] = PWMoutput[right_back] = 254
  #print "encoder_reading[left_front]:  " + str(encoder_reading[left_front])
  
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
   global rotary_counters, current_A, current_B, lock_rotary
                                       # read both of the switches
   Switch_A = GPIO.input(Enc_0_A)
   Switch_B = GPIO.input(Enc_0_B)
                                       # now check if state of A or B has changed
                                       # if not that means that bouncing caused it
   if current_A[left_front] == Switch_A and current_B[left_front] == Switch_B:      # Same interrupt as before (Bouncing)?
      return                                                    # ignore interrupt!

   current_A[left_front] = Switch_A                        # remember new state
   current_B[left_front] = Switch_B                        # for next bouncing check


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
   global rotary_counters, current_A, current_B, lock_rotary
                                       # read both of the switches
   Switch_A = GPIO.input(Enc_1_A)
   Switch_B = GPIO.input(Enc_1_B)
                                       # now check if state of A or B has changed
                                       # if not that means that bouncing caused it
   if current_A[right_front] == Switch_A and current_B[right_front] == Switch_B:      # Same interrupt as before (Bouncing)?
      return                                                    # ignore interrupt!

   current_A[right_front] = Switch_A                        # remember new state
   current_B[right_front] = Switch_B                        # for next bouncing check


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
   global rotary_counters, current_A, current_B, lock_rotary
                                       # read both of the switches
   Switch_A = GPIO.input(Enc_2_A)
   Switch_B = GPIO.input(Enc_2_B)
                                       # now check if state of A or B has changed
                                       # if not that means that bouncing caused it
   if current_A[left_back] == Switch_A and current_B[left_back] == Switch_B:      # Same interrupt as before (Bouncing)?
      return                                                    # ignore interrupt!

   current_A[left_back] = Switch_A                        # remember new state
   current_B[left_back] = Switch_B                        # for next bouncing check


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
   global rotary_counters, current_A, current_B, lock_rotary
                                       # read both of the switches
   Switch_A = GPIO.input(Enc_3_A)
   Switch_B = GPIO.input(Enc_3_B)
                                       # now check if state of A or B has changed
                                       # if not that means that bouncing caused it
   if current_A[right_back] == Switch_A and current_B[right_back] == Switch_B:      # Same interrupt as before (Bouncing)?
      return                                                    # ignore interrupt!

   current_A[right_back] = Switch_A                        # remember new state
   current_B[right_back] = Switch_B                        # for next bouncing check


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

  print "\n\n\n\n\n\nThe server is not running"
  
  global rotary_counters, lock_rotary, encoder_reading

  total_count = [0,0,0,0]
  new_counter = [0,0,0,0]

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



  print('reading from serial port %s...' % strPort),
  print('            plotting data...')

  
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
        drive(route[frameCnt][0],route[frameCnt+1][0], route[frameCnt][1],route[frameCnt+1][1])
        ser.write(encoderfeedback())

      
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
    for x_wheel in xrange(4):                              
      lock_rotary[x_wheel].acquire()               # get lock for rotary switch
      new_counter[x_wheel] = rotary_counters[x_wheel]      # get counter value
      rotary_counters[x_wheel] = 0                 # RESET IT TO 0
      lock_rotary[x_wheel].release()               # and release lock
               
      if (new_counter[x_wheel] !=0):               # Counter has CHANGED
         total_count[x_wheel] = total_count[x_wheel] + new_counter[x_wheel]

    if (cntSpeed > 10):
      for x_wheel in xrange(4):
        encoder_reading[x_wheel] = (total_count[x_wheel]/cntSpeed)
      print "enc:    " + str(int(encoder_reading[left_front]*100)) + "  " + str(int(encoder_reading[right_front]*100)) + "  " + str(int(encoder_reading[left_back]*100)) + "  " + str(int(encoder_reading[right_back]*100))
      cntSpeed = total_count[0] = total_count[1] = total_count[2] = total_count[3] = 0

  print('exiting.')
  

# call main
if __name__ == '__main__':
  main()
