#!/usr/bin/python
import picamera
from graphics import *
from PIL import Image
camera = picamera.PiCamera()
camera.resolution = (500, 500)
camera.exposure_mode = 'sports'
camera.capture('image.jpg')

pil = Image.open("image.jpg").convert('L')
#pil = pil.resize((500,500),Image.ANTIALIAS)
#enh = ImageEnhance.Contrast(pil)
#enh.enhance(1.7).show("30% more contrast")
pil.save('a.gif')
width, height = pil.size
raw = pil.tostring()

win = GraphWin('qr', 500, 500) 
pi = Line(Point(0,0),Point(10,10))
qrImage = IMAGE(Point(100,100), "a.gif")
qrImage.draw(win)

win.getMouse() #pause for click in window