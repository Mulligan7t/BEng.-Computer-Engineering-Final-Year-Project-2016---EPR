import socket

class Client():
   def __init__(self,Address=("127.0.0.1",5000)):
      self.s = socket.socket()
      try:
      	self.s.connect(Address)
      except Exception, e:
      	print "The server is not running"
      

TC=Client()