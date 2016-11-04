#!/usr/bin/python
import picamera
camera = picamera.PiCamera()
camera.resolution = (500, 500)
camera.exposure_mode = 'sports'
camera.capture('image.jpg')
