#!/usr/bin/python
from __future__ import division
from sys import argv
import zbar
from PIL import Image
from graphics import *
if len(argv) < 2: exit(1)

# create a reader
scanner = zbar.ImageScanner()

# configure the reader
scanner.parse_config('enable')

# obtain image data
pil = Image.open(argv[1]).convert('L')
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

win.getMouse() #pause for click in window


