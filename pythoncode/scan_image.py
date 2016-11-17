#!/usr/bin/python
from __future__ import division
from sys import argv
import zbar
from PIL import Image
from PIL import ImageEnhance
from graphics import *
import math
if len(argv) < 2: exit(1)

# create a reader
scanner = zbar.ImageScanner()

# configure the reader
scanner.parse_config('enable')

# obtain image data
pil = Image.open(argv[1]).convert('L')
#pil = pil.resize((500,500),Image.ANTIALIAS)
#enh = ImageEnhance.Contrast(pil)
#enh.enhance(1.7).show("30% more contrast")
pil.save('resized_image.jpg')
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


#print barloc[0][0]

# clean up
del(image)


if(True):
	win = GraphWin('qr', 500, 500)
	win.setBackground('white')

	i = 0
	while (i<3):
	    line = Line(Point(barloc[i][0],barloc[i][1]), Point(barloc[i+1][0], barloc[i+1][1]))
	    line.draw(win)
	    i = i + 1
	line = Line(Point(barloc[i][0],barloc[i][1]), Point(barloc[0][0], barloc[0][1]))
	line.draw(win)

	pt = Circle(Point(barloc[0][0],barloc[0][1]),20)
	pt.setFill('black')
	pt.setOutline('black')
	pt.draw(win)
	pt = Circle(Point(barloc[1][0],barloc[1][1]),5)
	pt.setFill('grey')
	pt.setOutline('grey')
	pt.draw(win) 
	pt = Circle(Point(barloc[3][0],barloc[3][1]),5)
	pt.setFill('grey')
	pt.setOutline('grey')
	pt.draw(win) 	

	x_center = float(barloc[0][0]+barloc[1][0]+barloc[2][0]+barloc[3][0])/4
	y_center = float(barloc[0][1]+barloc[1][1]+barloc[2][1]+barloc[3][1])/4

	pt = Circle(Point(x_center,y_center),5)
	pt.setOutline('blue')
	pt.setFill('blue')
	pt.draw(win) 

	x_dir = float(barloc[0][0]+barloc[3][0])/2
	y_dir = float(barloc[0][1]+barloc[3][1])/2


	line = Line(Point(x_center,y_center), Point(x_dir,y_dir))
	line.setWidth(3)
	line.setFill('blue')
	line.draw(win)

	pt = Circle(Point(x_dir,y_dir),3)
	pt.setFill('red')
	pt.setOutline('red')
	pt.draw(win) 

	qr_angle = math.degrees(math.atan2(y_center - y_dir, x_center - x_dir))-90
	print "qr angle: ", qr_angle


	win.getMouse() #pause for click in window


