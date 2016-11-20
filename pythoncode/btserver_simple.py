#!/usr/bin/python

from bluetooth import *
import threading

uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

server_sock = BluetoothSocket( RFCOMM )
server_sock.bind(("", 8))
server_sock.listen(1)

advertise_service( server_sock, "BT SERVER",
                   service_id = uuid,
                   service_classes = [ uuid, SERIAL_PORT_CLASS ],
                   profiles = [ SERIAL_PORT_PROFILE ], 
#                   protocols = [ OBEX_UUID ] 
                    )
print "Waiting for connection"

client_sock, client_info = server_sock.accept()
# print client_info, ": connection accepted"

wait_while = True
while wait_while:
    BT_route =  client_sock.recv(1024)
    if len(BT_route) == 0: break
    client_sock.send("ACK")
    wait_while = False

server_sock.close()

print "Route: ", BT_route
route_list = BT_route.split(' ')

BT_route_len = len(route_list) - 1
BT_route_list = []
for x in range(0, BT_route_len):
    BT_route_list.append((int(route_list[x].split(',')[0]),  int(route_list[x].split(',')[1])))
print BT_route_list



