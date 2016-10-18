
import RPi.GPIO as GPIO
import threading
from time import sleep

                  # GPIO Ports
Enc_0_A = 1              # Encoder input A: input GPIO 2 
Enc_0_B = 0                   # Encoder input B: input GPIO 3

Enc_1_A = 21             # Encoder input A: input GPIO 2 
Enc_1_B = 20                   # Encoder input B: input GPIO 3

Enc_2_A = 14              # Encoder input A: input GPIO 2 
Enc_2_B = 15                 # Encoder input B: input GPIO 3

Enc_3_A = 2              # Encoder input A: input GPIO 2 
Enc_3_B = 3                   # Encoder input B: input GPIO 3



Rotary_counter_0 = 0           # Start counting from 0
Current_0_A = 1               # Assume that rotary switch is not 
Current_0_B = 1               # moving while we init software
LockRotary_0 = threading.Lock()      # create lock for rotary switch


Rotary_counter_1 = 0           # Start counting from 0
Current_1_A = 1               # Assume that rotary switch is not 
Current_1_B = 1               # moving while we init software
LockRotary_1 = threading.Lock()      # create lock for rotary switch
   
Rotary_counter_2 = 0           # Start counting from 0
Current_2_A = 1               # Assume that rotary switch is not 
Current_2_B = 1               # moving while we init software
LockRotary_2 = threading.Lock()      # create lock for rotary switch

Rotary_counter_3 = 0           # Start counting from 0
Current_3_A = 1               # Assume that rotary switch is not 
Current_3_B = 1               # moving while we init software
LockRotary_3 = threading.Lock()      # create lock for rotary switch

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
      return                              # ignore interrupt!

   Current_0_A = Switch_A                        # remember new state
   Current_0_B = Switch_B                        # for next bouncing check


   if (Switch_A and Switch_B):                  # Both one active? Yes -> end of sequence
      LockRotary_0.acquire()                  # get lock 
      if A_or_B == Enc_0_B:                     # Turning direction depends on 
         Rotary_counter_0 += 1                  # which input gave last interrupt
      else:                              # so depending on direction either
         Rotary_counter_0 -= 1                  # increase or decrease counter
      LockRotary_0.release()                  # and release lock
   return                                 # THAT'S IT

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
      return                              # ignore interrupt!

   Current_1_A = Switch_A                        # remember new state
   Current_1_B = Switch_B                        # for next bouncing check


   if (Switch_A and Switch_B):                  # Both one active? Yes -> end of sequence
      LockRotary_1.acquire()                  # get lock 
      if A_or_B == Enc_1_B:                     # Turning direction depends on 
         Rotary_counter_1 += 1                  # which input gave last interrupt
      else:                              # so depending on direction either
         Rotary_counter_1 -= 1                  # increase or decrease counter
      LockRotary_1.release()                  # and release lock
   return                                 # THAT'S IT

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
      return                              # ignore interrupt!

   Current_2_A = Switch_A                        # remember new state
   Current_2_B = Switch_B                        # for next bouncing check


   if (Switch_A and Switch_B):                  # Both one active? Yes -> end of sequence
      LockRotary_2.acquire()                  # get lock 
      if A_or_B == Enc_2_B:                     # Turning direction depends on 
         Rotary_counter_2 += 1                  # which input gave last interrupt
      else:                              # so depending on direction either
         Rotary_counter_2 -= 1                  # increase or decrease counter
      LockRotary_2.release()                  # and release lock
   return                                 # THAT'S IT

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
      return                              # ignore interrupt!

   Current_3_A = Switch_A                        # remember new state
   Current_3_B = Switch_B                        # for next bouncing check


   if (Switch_A and Switch_B):                  # Both one active? Yes -> end of sequence
      LockRotary_3.acquire()                  # get lock 
      if A_or_B == Enc_3_B:                     # Turning direction depends on 
         Rotary_counter_3 += 1                  # which input gave last interrupt
      else:                              # so depending on direction either
         Rotary_counter_3 -= 1                  # increase or decrease counter
      LockRotary_3.release()                  # and release lock
   return                                 # THAT'S IT
 

# Main loop. Demonstrate reading, direction and speed_0 of turning left/rignt
def main():
   global Rotary_counter_0, LockRotary_0, Rotary_counter_1, LockRotary_1,Rotary_counter_2, LockRotary_2, Rotary_counter_3, LockRotary_3
   

   TotalCount_0 = 0                           # Current TotalCount_0   
   NewCounter_0 = 0                        # for faster reading with locks           
   speed_0 = 0
   TotalCount_1 = 0                           # Current TotalCount_1   
   NewCounter_1 = 0                        # for faster reading with locks           
   speed_1 = 0
   TotalCount_2 = 0                           # Current TotalCount_2   
   NewCounter_2 = 0                        # for faster reading with locks           
   speed_2 = 0
   TotalCount_3 = 0                           # Current TotalCount_3   
   NewCounter_3 = 0                        # for faster reading with locks           
   speed_3 = 0
   cnt = 0 


   init()                              # Init interrupts, GPIO, ...
            
   while True :                        # start test 
      sleep(0.1)                        # sleep 100 msec
      cnt = cnt +1
                                    # because of threading make sure no thread
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         # changes value until we get them
                                    # and reset them
                                    
      LockRotary_0.acquire()               # get lock for rotary switch
      NewCounter_0 = Rotary_counter_0         # get counter value
      Rotary_counter_0 = 0                  # RESET IT TO 0
      LockRotary_0.release()               # and release lock
               
      if (NewCounter_0 !=0):               # Counter has CHANGED
         TotalCount_0 = TotalCount_0 + NewCounter_0

      LockRotary_1.acquire()               # get lock for rotary switch
      NewCounter_1 = Rotary_counter_1         # get counter value
      Rotary_counter_1 = 0                  # RESET IT TO 0
      LockRotary_1.release()               # and release lock
               
      if (NewCounter_1 !=0):               # Counter has CHANGED
         TotalCount_1 = TotalCount_1 + NewCounter_1

      LockRotary_2.acquire()               # get lock for rotary switch
      NewCounter_2 = Rotary_counter_2         # get counter value
      Rotary_counter_2 = 0                  # RESET IT TO 0
      LockRotary_2.release()               # and release lock
               
      if (NewCounter_2 !=0):               # Counter has CHANGED
         TotalCount_2 = TotalCount_2 + NewCounter_2

      LockRotary_3.acquire()               # get lock for rotary switch
      NewCounter_3 = Rotary_counter_3         # get counter value
      Rotary_counter_3 = 0                  # RESET IT TO 0
      LockRotary_3.release()               # and release lock
               
      if (NewCounter_3 !=0):               # Counter has CHANGED
         TotalCount_3 = TotalCount_3 + NewCounter_3

                                 
      if (cnt > 10):
        speed_0 = (TotalCount_0 /cnt)
        speed_1 = (TotalCount_1 /cnt)
        speed_2 = (TotalCount_2 /cnt)
        speed_3 = (TotalCount_3 /cnt)
        print speed_0, speed_1, speed_2, speed_3
        cnt = 0
        TotalCount_0 = 0
        TotalCount_1 = 0
        TotalCount_2 = 0
        TotalCount_3 = 0


# start main demo function
main()
