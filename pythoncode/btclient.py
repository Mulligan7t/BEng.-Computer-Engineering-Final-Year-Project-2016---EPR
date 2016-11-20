#!/usr/bin/python

# Requirements:
# sudo aptitude install python-bluetooth

# Information Sources: 
# http://code.google.com/p/pybluez/source/browse/trunk/examples/simple/rfc...
# http://people.csail.mit.edu/albert/bluez-intro/x290.html#py-rfcomm-clien...

import bluetooth
import sys
import threading

#uuid = "fa87c0d0-afac-11de-8a39-0800200c9a66"
uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
addr = "B8:27:EB:FE:EF:75"

print "uuid: %s" % uuid
if len(sys.argv) < 2:
    print "Using default:  B8:27:EB:FE:EF:75"
else:
    addr = sys.argv[1]
    print "Searching for BluetoothChat on %s" % addr

# search for the BluetoothChat service
service_matches = bluetooth.find_service( uuid = uuid, address = addr )

if len(service_matches) == 0:
    print "couldn't find the BluetoothChat service =("
    sys.exit(0)

first_match = service_matches[0]
port = first_match["port"]
name = first_match["name"]
host = first_match["host"]

print "connecting to \"%s\" on %s" % (name, host)

# Create the client socket
sock=bluetooth.BluetoothSocket( bluetooth.RFCOMM )
sock.connect((host, port))

class receiverThread(threading.Thread):
    def __init__ (self,sock):
        threading.Thread.__init__(self)
        self.sock = sock
    def run(self):
        while True:
            data = self.sock.recv(1024)
            if len(data) == 0: break
            print "C received [%s]" % data
            
receiver = receiverThread(sock)
receiver.setDaemon(True)
receiver.start()
print "connected - type stuff:"
while True:
    data = raw_input()
    if len(data) == 0: break
    sock.send(data)
    #print "C sent [%s]" % data

sock.close()