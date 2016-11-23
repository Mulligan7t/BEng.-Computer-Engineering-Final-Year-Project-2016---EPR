#!/usr/bin/python

import bluetooth
import sys
import threading
import Tkinter as tk
import socket, time,os, random
# Set number of rows and columns
ROWS = 4
COLS = 4
global route
route= ""
routelist= []

# A0 A1 A2
# B0 B1 B2
# C0 C1 C2

# Create a grid of None to store the references to the tiles
tiles = [[None for _ in range(COLS)] for _ in range(ROWS)]

def callback(event):
    # Get rectangle diameters
    col_width = c.winfo_width()/COLS
    row_height = c.winfo_height()/ROWS
    # Calculate column and row number
    col = event.x//col_width
    row = event.y//row_height
    # If the tile is not filled, create a rectangle
    if not tiles[row][col]:
        tiles[row][col] = c.create_rectangle(col*col_width, row*row_height, (col+1)*col_width-10, (row+1)*row_height-10, fill="grey")
        canvas_id = c.create_text(100, 100, anchor="c",)
        c.itemconfig(canvas_id, text="A0", font=("Purisa", 28))


        canvas_id = c.create_text(300, 100, anchor="c")
        c.itemconfig(canvas_id, text="A1", font=("Purisa", 28))

        canvas_id = c.create_text(500, 100, anchor="c")
        c.itemconfig(canvas_id, text="A2", font=("Purisa", 28))

        canvas_id = c.create_text(700, 100, anchor="c")
        c.itemconfig(canvas_id, text="A3", font=("Purisa", 28))

        canvas_id = c.create_text(100, 300, anchor="c",)
        c.itemconfig(canvas_id, text="B0", font=("Purisa", 28))

        canvas_id = c.create_text(300, 300, anchor="c")
        c.itemconfig(canvas_id, text="B1", font=("Purisa", 28))

        canvas_id = c.create_text(500, 300, anchor="c")
        c.itemconfig(canvas_id, text="B2", font=("Purisa", 28))

        canvas_id = c.create_text(700, 300, anchor="c")
        c.itemconfig(canvas_id, text="B3", font=("Purisa", 28))

        canvas_id = c.create_text(100, 500, anchor="c",)
        c.itemconfig(canvas_id, text="C0", font=("Purisa", 28))

        canvas_id = c.create_text(300, 500, anchor="c")
        c.itemconfig(canvas_id, text="C1", font=("Purisa", 28))

        canvas_id = c.create_text(500, 500, anchor="c")
        c.itemconfig(canvas_id, text="C2", font=("Purisa", 28))

        canvas_id = c.create_text(700, 500, anchor="c")
        c.itemconfig(canvas_id, text="C3", font=("Purisa", 28))


        canvas_id = c.create_text(100, 700, anchor="c",)
        c.itemconfig(canvas_id, text="D0", font=("Purisa", 28))

        canvas_id = c.create_text(300, 700, anchor="c")
        c.itemconfig(canvas_id, text="D1", font=("Purisa", 28))

        canvas_id = c.create_text(500, 700, anchor="c")
        c.itemconfig(canvas_id, text="D2", font=("Purisa", 28))

        canvas_id = c.create_text(700, 700, anchor="c")
        c.itemconfig(canvas_id, text="D3", font=("Purisa", 28))

    # If the tile is filled, delete the rectangle and clear the reference
    else:
        c.delete(tiles[row][col])
        tiles[row][col] = None
    nu = 65 + row
    print str(unichr(nu)) + " " + str(col) 
    setroute(row,col)

def setroute (a, b):
    global route, routelist
    routelist.append((a,b))
    route += str(a) + ","
    route += str(b) + " "
    print routelist

def WaitForConnection():
    global s
    Client, Addr=(s.accept())
    print('Connection achieved.')


# Create the window, a canvas and the mouse click event binding


root = tk.Tk()
c = tk.Canvas(root, width=COLS*200, height=ROWS*200, borderwidth=10, background='white')

canvas_id = c.create_text(100, 100, anchor="c",)
c.itemconfig(canvas_id, text="A0", font=("Purisa", 28))

canvas_id = c.create_text(300, 100, anchor="c")
c.itemconfig(canvas_id, text="A1", font=("Purisa", 28))

canvas_id = c.create_text(500, 100, anchor="c")
c.itemconfig(canvas_id, text="A2", font=("Purisa", 28))

canvas_id = c.create_text(700, 100, anchor="c")
c.itemconfig(canvas_id, text="A3", font=("Purisa", 28))

canvas_id = c.create_text(100, 300, anchor="c",)
c.itemconfig(canvas_id, text="B0", font=("Purisa", 28))

canvas_id = c.create_text(300, 300, anchor="c")
c.itemconfig(canvas_id, text="B1", font=("Purisa", 28))

canvas_id = c.create_text(500, 300, anchor="c")
c.itemconfig(canvas_id, text="B2", font=("Purisa", 28))

canvas_id = c.create_text(700, 300, anchor="c")
c.itemconfig(canvas_id, text="B3", font=("Purisa", 28))

canvas_id = c.create_text(100, 500, anchor="c",)
c.itemconfig(canvas_id, text="C0", font=("Purisa", 28))

canvas_id = c.create_text(300, 500, anchor="c")
c.itemconfig(canvas_id, text="C1", font=("Purisa", 28))

canvas_id = c.create_text(500, 500, anchor="c")
c.itemconfig(canvas_id, text="C2", font=("Purisa", 28))

canvas_id = c.create_text(700, 500, anchor="c")
c.itemconfig(canvas_id, text="C3", font=("Purisa", 28))


canvas_id = c.create_text(100, 700, anchor="c",)
c.itemconfig(canvas_id, text="D0", font=("Purisa", 28))

canvas_id = c.create_text(300, 700, anchor="c")
c.itemconfig(canvas_id, text="D1", font=("Purisa", 28))

canvas_id = c.create_text(500, 700, anchor="c")
c.itemconfig(canvas_id, text="D2", font=("Purisa", 28))

canvas_id = c.create_text(700, 700, anchor="c")
c.itemconfig(canvas_id, text="D3", font=("Purisa", 28))

# row = 0
# col = 0
# col_width = c.winfo_width()/COLS
# row_height = c.winfo_height()/ROWS    

# tiles[row][col] = c.create_rectangle(col*col_width, row*row_height, (col+1)*col_width-10, (row+1)*row_height-10, fill="black")


c.pack()
c.bind("<Button-1>", callback)


root.title("Select a route")
root.mainloop()


#==============================================================================================

print "Waiting for BT Server"
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
        global wait
        while True:
            data = self.sock.recv(1024)
            if len(data) == 0: break
            print "C received [%s]" % data
            wait = False


wait = True            
receiver = receiverThread(sock)
receiver.setDaemon(True)
receiver.start()

print route
sock.send(str(route))

while wait:
    pass

if(False):
    print "connected - type stuff:"
    while True:
        data = raw_input()
        if len(data) == 0: break
        sock.send(data)
        #print "C sent [%s]" % data

print "END"
sock.close()

