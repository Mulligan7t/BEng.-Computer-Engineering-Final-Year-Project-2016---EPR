#!/usr/bin/python

# Requirements:
# sudo aptitude install python-bluetooth

# Information Sources: 
# http://code.google.com/p/pybluez/source/browse/trunk/examples/simple/rfc...
# http://people.csail.mit.edu/albert/bluez-intro/x290.html#py-rfcomm-serve...

from bluetooth import *
import threading

name = "BluetoothConnection"
#uuid = "fa87c0d0-afac-11de-8a39-0800200c9a66"
uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

print "uuid: %s" % uuid
server_sock = BluetoothSocket( RFCOMM )
server_sock.bind(("", 8))
server_sock.listen(1)
port = server_sock.getsockname()[1]

advertise_service( server_sock, "BT SERVER",
                   service_id = uuid,
                   service_classes = [ uuid, SERIAL_PORT_CLASS ],
                   profiles = [ SERIAL_PORT_PROFILE ], 
#                   protocols = [ OBEX_UUID ] 
                    )
print "Waiting for connection on RFCOMM channel %d" % port

class echoThread(threading.Thread):
    def __init__ (self,sock,client_info):
        threading.Thread.__init__(self)
        self.sock = sock
        self.client_info = client_info

    def run(self):
        global continuerun
        try:
            wait_while = True
            while wait_while:
                data = self.sock.recv(1024)
                if len(data) == 0: break
                print "received [%s]" % data
                data = data + "cata"
                self.sock.send("ACK")
                wait_while = False
                continuerun = False

                # data = raw_input()
                # if len(data) == 0: break
                # #print "typed and sent [%s]" % data
                # self.sock.send(data)
                # print self.client_info, ": sent [%s]" % data
        except IOError:
            pass
        self.sock.close()
        print "disconnected"

continuerun = True

client_sock, client_info = server_sock.accept()
print client_info, ": connection accepted"
echo = echoThread(client_sock, client_info)
echo.setDaemon(True)
echo.start()

while continuerun:
    pass

server_sock.close()
print "all done"
