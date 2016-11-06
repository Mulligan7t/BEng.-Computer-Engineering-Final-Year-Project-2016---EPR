#!/usr/bin/python
from __future__ import division
from sys import argv
import zbar
from PIL import Image
from graphics import *
if len(argv) < 2: exit(1)
import sys, serial, argparse
import numpy as np
import time
from datetime import datetime
from collections import deque
import socket
import RPi.GPIO as GPIO
import threading
import math
from smbus import SMBus

busNum = 1
b = SMBus(busNum)
LSM = 0x1d
LSM_WHOAMI = 0b1001001 #Device self-id

#Control register addresses -- from LSM303D datasheet
CTRL_0 = 0x1F #General settings
CTRL_1 = 0x20 #Turns on accelerometer and configures data rate
CTRL_2 = 0x21 #Self test accelerometer, anti-aliasing accel filter
CTRL_3 = 0x22 #Interrupts
CTRL_4 = 0x23 #Interrupts
CTRL_5 = 0x24 #Turns on temperature sensor
CTRL_6 = 0x25 #Magnetic resolution selection, data rate config
CTRL_7 = 0x26 #Turns on magnetometer and adjusts mode
#Registers holding twos-complemented MSB and LSB of magnetometer readings -- from LSM303D datasheet
MAG_X_LSB = 0x08 # x
MAG_X_MSB = 0x09
MAG_Y_LSB = 0x0A # y
MAG_Y_MSB = 0x0B
MAG_Z_LSB = 0x0C # z
MAG_Z_MSB = 0x0D
#Registers holding twos-complemented MSB and LSB of magnetometer readings -- from LSM303D datasheet
ACC_X_LSB = 0x28 # x
ACC_X_MSB = 0x29
ACC_Y_LSB = 0x2A # y
ACC_Y_MSB = 0x2B
ACC_Z_LSB = 0x2C # z
ACC_Z_MSB = 0x2D
#Registers holding 12-bit right justified, twos-complemented temperature data -- from LSM303D datasheet
TEMP_MSB = 0x05
TEMP_LSB = 0x06

                          # GPIO Ports
Enc_1_A = 21              # Encoder input A: input GPIO 21
Enc_1_B = 20              # Encoder input B: input GPIO 20
Enc_0_A = 1               # Encoder input A: input GPIO 1 
Enc_0_B = 0               # Encoder input B: input GPIO 0
  
#Enc_2_A = 2               # Encoder input A: input GPIO 2 
#Enc_2_B = 3               # Encoder input B: input GPIO 3

Enc_2_A = 6              # over writes because required for I2C
Enc_2_B = 5             # over writes because required for I2C

Enc_3_A = 14              # Encoder input A: input GPIO 14 
Enc_3_B = 15              # Encoder input B: input GPIO 15

#array names
left_front = 0
right_front = 1
left_back = 2
right_back = 3

lock_rotary = [threading.Lock(),threading.Lock(),threading.Lock(),threading.Lock()]     # create lock for rotary switch
rotary_counters = [0,0,0,0]

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
Kd = 0.3

integral_error = [0,0,0,0]
prev_error = [0,0,0,0]

min_interia = 200 #minimum PWM to break intertia and start turning
min_dynamic = 50 #lower than this and will stall even after rotation has begun

scaling_factor = 1000

magnetic_heading_LSM303 = 0

route_counter = 0 #what position on the route are we on now.

ext_var = 0
stat = 0
prev_stat = 0
prev_time = datetime.now()
current_time = datetime.now()

def drive(coY0, coY1, coX0, coX1):
  global motor_setpoint, magnetic_heading_LSM303, ext_var,route_counter

  coXdiff = coX1-coX0
  coYdiff = coY1-coY0
  #print coXdiff
  #print coYdiff

  #  linearX = coXdiff*scaling_factor
  #  linearY = coYdiff*scaling_factor
  #  angularZ = 0

  if ext_var is None:
    desired_heading = 90.0 - math.degrees(math.atan2(coYdiff,coXdiff))
    if(desired_heading< 0):
      desired_heading= desired_heading+ 360    
  else:
    desired_heading = int(ext_var)
  

  # if(desired_heading>180):
  #   desired_heading = desired_heading - 360

  #print "ORIG:       ", magnetic_heading_LSM303
  magnetic_heading_LSM303_360 = magnetic_heading_LSM303
  
  # if(magnetic_heading_LSM303>180):
  #   magnetic_heading_LSM303_360 = magnetic_heading_LSM303 - 360

  heading_adjustment = magnetic_heading_LSM303_360 - desired_heading
  
  if (math.fabs(heading_adjustment) < 1):
    heading_adjustment = 0
    #route_counter = route_counter + 1 #move to next qr code because robot is facing correct direction
    print "route counter   ", route_counter

  if (heading_adjustment>180):
    heading_adjustment = heading_adjustment - 360

  #print "adj_head = mag_head_LSM - desired_heading"
  #print heading_adjustment, " = " , magnetic_heading_LSM303_360, " - " , desired_heading

  accuracy = 1
  left_dir = right_dir = 0 #stop all

  if(math.fabs(heading_adjustment)<=accuracy):
      left_dir = right_dir = 0 #stop all
      #print "first"
  elif (heading_adjustment<accuracy):
    left_dir = 1*math.fabs(heading_adjustment)/100
    right_dir = -1*math.fabs(heading_adjustment)/100
    #print "second"
  elif(heading_adjustment>accuracy):
    left_dir = -1*math.fabs(heading_adjustment)/100
    right_dir = 1*math.fabs(heading_adjustment)/100
    #print "third"
    
  motor_setpoint[left_front] = 255 * left_dir
  motor_setpoint[left_back] = 255 * left_dir


  motor_setpoint[right_front] = 255 * right_dir
  motor_setpoint[right_back] = 255 * right_dir


  print "heading adj--------------------------------------------------"
  print heading_adjustment
  angularZ = math.radians(heading_adjustment)
  print "current heading: "
  print magnetic_heading_LSM303_360 

  if(False):
    linearX = 0
    linearY = 0 #next
    angularZ = 0
    print "heading adj--------------------------------------------------"
    print heading_adjustment
    angularZ = math.radians(heading_adjustment)
    print "current heading: "
    print magnetic_heading_LSM303_360
    motor_setpoint[left_front] = (1/WHEEL_RADIUS) * (linearX - linearY - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularZ) * speedcalib
    motor_setpoint[right_front] = (1/WHEEL_RADIUS) * (linearX + linearY + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularZ) * speedcalib
    motor_setpoint[left_back] = (1/WHEEL_RADIUS) * (linearX + linearY - (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularZ) * speedcalib
    motor_setpoint[right_back] = (1/WHEEL_RADIUS) * (linearX - linearY + (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH)*angularZ) * speedcalib

  if (False):
    all_dir = 1
    motor_setpoint[left_front] = 255 * all_dir
    motor_setpoint[right_front] = 255 * all_dir
    motor_setpoint[left_back] = 255 * all_dir
    motor_setpoint[right_back] = 255 * all_dir

  encoderRate = 9


  for x_wheel in xrange(4):
    motor_setpoint[x_wheel] = motor_setpoint[x_wheel]//encoderRate
    if(math.fabs(motor_setpoint[x_wheel])<=1):
      motor_setpoint[x_wheel] = 0

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
    current_error[x_wheel] = motor_setpoint[x_wheel]-encoder_reading[x_wheel]

  for x_wheel in xrange(4):
    integral_error[x_wheel] = integral_error[x_wheel] + current_error[x_wheel]

  for x_wheel in xrange(4):
    if((current_error[x_wheel] < 2) or (integral_error[x_wheel] > 254)):
      integral_error[x_wheel] = 0


  for x_wheel in xrange(4):
    derivative_error[x_wheel] = current_error[x_wheel] - prev_error[x_wheel]
    prev_error[x_wheel] = current_error[x_wheel]
    PWMoutput[x_wheel] = PWMoutput[x_wheel] + (Kp*current_error[x_wheel] + Ki*integral_error[x_wheel] + Kd*derivative_error[x_wheel])
    #print "encoder_reading[x_wheel]: " + str(encoder_reading[x_wheel]),
    #print "  integral_error[x_wheel]: " + str(integral_error[x_wheel]),
    #print "  derivative_error[x_wheel]: " + str(derivative_error[x_wheel])
  print "encoder_reading[left_front]: " + str(encoder_reading[left_front]),
  print "  current_error[left_front]: " + str(current_error[left_front]),
  print "  integral_error[left_front]: " + str(integral_error[left_front]),
  print "  derivative_error[left_front]: " + str(derivative_error[left_front])
        
  for x_wheel in xrange(4):  
    if(encoder_reading[x_wheel]==0 and math.fabs(motor_setpoint[x_wheel]) > 0):
      #motor is not moving
      #motor set to run forward or backward
      PWMoutput[x_wheel] = math.copysign(min_interia, motor_setpoint[x_wheel]) # return value of min_interia with the sign of motor_setpoint[x_wheel]
      #print "  ",
      #print x_wheel,
      #print "  ",

    if(math.fabs(motor_setpoint[x_wheel]) > 0 and math.fabs(encoder_reading[x_wheel])>1.0 and math.fabs(PWMoutput[x_wheel]) <min_dynamic):
      #motor set to run forward or backward
      #motor is running at least 1.0 forward or backward
      #PWMoutput is lower than min_dynamic
      #THEN set PWMoutput to min_dynamic
      PWMoutput[x_wheel] = math.copysign(min_dynamic, motor_setpoint[x_wheel]) # return value of min_dynamic with the sign of motor_setpoint[x_wheel]

  
    if(math.fabs(PWMoutput[x_wheel])>254):
      PWMoutput[x_wheel] = math.copysign(254, motor_setpoint[x_wheel]) # return value of 254 with the sign of motor_setpoint[x_wheel]
    
    if(encoder_reading[x_wheel] ==0 and math.fabs(motor_setpoint[x_wheel]) <= 1 or motor_setpoint[x_wheel] == 0):
      motor_setpoint[x_wheel] = 0
      PWMoutput[x_wheel] = 0
  print


  #PWMoutput[left_front] = PWMoutput[right_front] = PWMoutput[left_back] = PWMoutput[right_back] = 254
  print "encoder  "+ str(encoder_reading[0])+ " " + str(encoder_reading[1])+" " + str(encoder_reading[2])+ " " +str(encoder_reading[3])
  
  
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
  #pil = pil.resize((500,500),Image.ANTIALIAS)
  #pil.save('resized_image.jpg')
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

  try:
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

  except Exception, e:
    print "Exception: ", e
  else:
    pass
  finally:
    pass
 

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

def twos_comp_combine(msb, lsb):
    twos_comp = 256*msb + lsb
    if twos_comp >= 32768:
        return twos_comp - 65536
    else:
        return twos_comp

def init_LSM303D():
  if b.read_byte_data(LSM, 0x0f) == LSM_WHOAMI:
      print 'LSM303D detected successfully.'
  else:
      print 'No LSM303D detected on bus '+str(busNum)+'.'
  b.write_byte_data(LSM, CTRL_1, 0b1010111) # enable accelerometer, 50 hz sampling
  b.write_byte_data(LSM, CTRL_2, 0x00) #set +/- 2g full scale
  b.write_byte_data(LSM, CTRL_5, 0b01100100) #high resolution mode, thermometer off, 6.25hz ODR
  b.write_byte_data(LSM, CTRL_6, 0b00100000) # set +/- 4 gauss full scale
  b.write_byte_data(LSM, CTRL_7, 0x00) #get magnetometer out of low power mode

def read_LSM303D():
  global magnetic_heading_LSM303
  magx = twos_comp_combine(b.read_byte_data(LSM, MAG_X_MSB), b.read_byte_data(LSM, MAG_X_LSB))
  magy = twos_comp_combine(b.read_byte_data(LSM, MAG_Y_MSB), b.read_byte_data(LSM, MAG_Y_LSB))
  magz = twos_comp_combine(b.read_byte_data(LSM, MAG_Z_MSB), b.read_byte_data(LSM, MAG_Z_LSB))
  # from ~/.minimu9-ahrs-cal


  minimumx = -1583  
  maximumx = 1970 
  minimumy = -1608  
  maximumy = 1810 
  minimumz = -1369 
  maximumz = 1988

  magx = (float)(magx - minimumx) / (maximumx - minimumx) * 2 - 1
  magy = (float)(magy - minimumy) / (maximumy - minimumy) * 2 - 1
  magz = (float)(magz - minimumz) / (maximumz - minimumz) * 2 - 1

  #print "magx", magx
  #print "magy", magy
  magnetic_heading_LSM303 = math.atan2(magy,magx)

  declinationAngle =  0 - math.radians(150) #150
  magnetic_heading_LSM303 += declinationAngle
  
  # Correct for when signs are reversed.
  if(magnetic_heading_LSM303 < 0):
    magnetic_heading_LSM303 = math.fsum([magnetic_heading_LSM303, 2*math.pi])
  
  # Check for wrap due to addition of declination.
  if(magnetic_heading_LSM303 > 2*math.pi):
    #magnetic_heading_LSM303 -= 2*math.pi
    magnetic_heading_LSM303 = math.fsum([magnetic_heading_LSM303, -2*math.pi])

  magnetic_heading_LSM303 = math.degrees(magnetic_heading_LSM303)

  accx = twos_comp_combine(b.read_byte_data(LSM, ACC_X_MSB), b.read_byte_data(LSM, ACC_X_LSB))
  accy = twos_comp_combine(b.read_byte_data(LSM, ACC_Y_MSB), b.read_byte_data(LSM, ACC_Y_LSB))
  accz = twos_comp_combine(b.read_byte_data(LSM, ACC_Z_MSB), b.read_byte_data(LSM, ACC_Z_LSB))
  
  #print "magnetic heading LSM303D: ", magnetic_heading_LSM303
  #print "Magnetic field (x, y, z):", magx, magy, print
  # magz "Acceleration   (x, y, z):", accx, accy, accz
  #print ""

# main() function
def main():
  
  try:  


    #  camqr()

    #  Address=("127.0.0.1",5000)
    #  s = socket.socket()
    #  try:
    #    s.connect(Address)
    #  except Exception, e:
    
    init_LSM303D()

    print "The server is not running"
    
    global rotary_counters, lock_rotary, encoder_reading, magnetic_heading_LSM303, route_counter, ext_var, prev_stat, stat, prev_time, current_time

    total_count = [0,0,0,0]
    new_counter = [0,0,0,0]

    cntSpeed = 0 

    init()                              # Init interrupts, GPIO, ...


    # create parser
    parser = argparse.ArgumentParser(description="QR serial")
    # add expected arguments
    parser.add_argument('--port', dest='port', required=True)
    parser.add_argument('-ext_var', dest='ext_var', required=False)

    # parse args
    args = parser.parse_args()
    
    #strPort = '/dev/tty.usbserial-A7006Yqh'
    #strPort = 'COM8'
    strPort = args.port
    ext_var = args.ext_var


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

    print route
    print 




    print('reading from serial port %s...' % strPort),
    print('            plotting data...')

    
    frameCnt = 0
    while True:
      prev_stat = stat
      stat = os.stat("image.jpg").st_mtime
      if (stat != prev_stat):
        print "----------------------------------------------------------------------------------------------------------------------------"
        camqr()


      if route_counter < routelen:
          drive(route[route_counter][0],route[route_counter+1][0], route[route_counter][1],route[route_counter+1][1])
          ser.write(encoderfeedback())

      try:
        line = ser.readline()
        #print "line: ",
        #print line,
        
        data = [float(val) for val in line.split()]
        # print data
        
        
        if(len(data) == 10):
          #magnetic_heading = data[8]
          #print "MAG"
          #print magnetic_heading
          #print data[8]
          #print data[0]
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
          GPIO.cleanup() # this ensures a clean exit
          raise SystemExit

      read_LSM303D() #read data from second acc and mag

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
      
      prev_time = current_time
      current_time = datetime.now()
      if (current_time.microsecond - prev_time.microsecond > 24000):
        print "delta time"
        print current_time.microsecond - prev_time.microsecond
        for x_wheel in xrange(4):
          encoder_reading[x_wheel] = ((total_count[x_wheel]/cntSpeed)*5)
        #print "enc:   " + str(int(encoder_reading[left_front]*100)) + " " + str(int(encoder_reading[right_front]*100)) + " " + str(int(encoder_reading[left_back]*100)) + " " + str(int(encoder_reading[right_back]*100))
        print "enc:   " + str(encoder_reading[left_front]) + " " + str(encoder_reading[right_front]) + " " + str(encoder_reading[left_back]) + " " + str(encoder_reading[right_back])
        cntSpeed = total_count[0] = total_count[1] = total_count[2] = total_count[3] = 0

    print('exiting.')
  except KeyboardInterrupt:  
      # here you put any code you want to run before the program   
      # exits when you press CTRL+C  
      print "exiting cleanly============================================="
        
  finally:  
      GPIO.cleanup() # this ensures a clean exit

# call main
if __name__ == '__main__':
  main()
