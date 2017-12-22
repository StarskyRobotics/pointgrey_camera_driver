#!/usr/bin/env python
#
# Select Blackfly product family and then download FlyCapture 2.x SDK and PyCapture 2.x SDK for your OS
# https://www.ptgrey.com/support/downloads
# user: code@clearpathrobotics.com
# pw: uNjRxoH6NMsJvi6hyPCH
#

from PyCapture2 import *
import time

def setProp(cam, proptype, autoset, value):
    pinfo = cam.getPropertyInfo(proptype)
    if pinfo.present:
        prop = Property()
        prop.type = proptype
        prop.autoManualMode = autoset and pinfo.autoSupported
        prop.absControl = pinfo.absValSupported
        prop.onOff = pinfo.onOffSupported
        prop.absValue = value
        cam.setProperty(prop)
    else:
        print("Property %d not available" % proptype)


def cam_setup(serial):
	bus = BusManager()
	uID = bus.getCameraFromSerialNumber(serial) 
	cam = GigECamera()
	print("Connecting to Camera...")
	cam.connect(uID)
	camInfo = cam.getCameraInfo()
	print("Serial: %d model: %s sensor: %s resolution: %s" % (camInfo.serialNumber, camInfo.modelName, camInfo.sensorInfo, camInfo.sensorResolution))


        print("Setting packet size and delay")
        cam.setGigEProperty(propType=GIGE_PROPERTY_TYPE.GIGE_PACKET_SIZE, value=1400)
        cam.setGigEProperty(propType=GIGE_PROPERTY_TYPE.GIGE_PACKET_DELAY, value=1000)

	print("Querying GigE image setting information...")
	imgSetInfo = cam.getGigEImageSettingsInfo()
	imgSet = GigEImageSettings()

	imgSet.offsetX = 0
	imgSet.offsetY = 0
	imgSet.height = imgSetInfo.maxHeight
	imgSet.width = imgSetInfo.maxWidth
	imgSet.pixelFormat = PIXEL_FORMAT.RAW8

	print("Setting GigE image settings...")
	cam.setGigEImageSettings(imgSet)
	cam.setEmbeddedImageInfo(timestamp=True)

        setProp(cam, PROPERTY_TYPE.FRAME_RATE, False, 30)
        setProp(cam, PROPERTY_TYPE.AUTO_EXPOSURE, True, 0)
        setProp(cam, PROPERTY_TYPE.GAIN, True, 0)

        # Taken from https://github.com/ros-drivers/pointgrey_camera_driver/blob/master/pointgrey_camera_driver/src/PointGreyCamera.cpp#L594
        white_balance_addr = 0x80c
        enable = 1 << 31
        value = 1 << 25
        blue = 800
        red = 550
        value |= 1 << 24
        value |= blue << 12 | red
        cam.writeRegister(white_balance_addr, enable)
        cam.writeRegister(white_balance_addr, value)

	print("Starting image capture...")
	cam.startCapture()

	return cam

US_PER_S=1000000.0

def capture_raw(cam):
        print("Grabbing image")
        image = cam.retrieveBuffer()

        ts = image.getTimeStamp()
        ts = ts.seconds + ts.microSeconds / US_PER_S
        print("Timestamp: %.3f" % ts)

        print("Returning raw data")
	data = image.getData() 
        return data

def measure_fps(cam):
        print("Measuring FPS")
        image = cam.retrieveBuffer()
        t = time.time()
        ok = fail = 0
        for i in range(0, 30):
            try:
                image = cam.retrieveBuffer()
                ok += 1
            except:
                fail += 1

        print("OK: %d Failed: %d FPS: %.1f" % (ok, fail, ok / (time.time()-t)))
        

def capture_png(cam):
        print("Grabbing image")
        image = cam.retrieveBuffer()
        print("Saving to test.png")
	newimg = image.convert(PIXEL_FORMAT.RGB)
	newimg.save("test.png", IMAGE_FILE_FORMAT.PNG)

def cleanup(cam):
        print("Shutting down")
	cam.stopCapture()
	cam.disconnect()

cam = cam_setup(serial=16452901)
data = capture_raw(cam)
print("Raw data contains %d bytes" % len(data))
capture_png(cam)
measure_fps(cam)
cleanup(cam)
