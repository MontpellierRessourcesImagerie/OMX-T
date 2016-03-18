'''
A very basic module to test the Zyla camera
'''
from AndorZyla import AndorBase, AndorZyla
from __builtin__ import str
import memoryHandler

print('Importing cameras...')

from AndorSDK3Camera import *

print('Cameras Imported')




numCameras = GetNumCameras()
print(str(numCameras) + ' cameras were detected')

softwareVersion = GetSoftwareVersion().value
print('Software version is: ' + softwareVersion)

camerasList = []

# for i in range(numCameras):
#     camerasList.append(AndorBase(i))


camerasList.append(AndorZyla(0))

for camera in camerasList:
    print('Initialising camera...')
    camera.Init()
    print('Camera model: ' + camera.getCameraModel())
    print('Camera serial nr: ' + camera.getSerialNumber())
    print('Current trigger mode: ' + str(camera.getTrigger()))
    print('Changing trigger mode to 5...')
    camera.setTrigger(5)
    print('Current trigger mode: ' + str(camera.getTrigger()))
    print('Changing trigger mode back to 0...')
    camera.setTrigger(0)
    print('Current trigger mode: ' + str(camera.getTrigger()))
    print('Current exposure time: ' + str(camera.getExposureTime()))
    print('Changing exposure to .5...')
    camera.setExposureTime(0.5)
    print('Current exposure time: ' + str(camera.getExposureTime()))
    print('The minimum exposure time is: ' + str(camera.getMinExposureTime()))
    if camera.getCameraModel() != 'SIMCAM CMOS':
        print('AOI size is: ' + str(camera.AOIWidth.getValue()) + 'x' + str(camera.AOIHeight.getValue()))
        print('Changing AOI to 512x512...')
        camera.setCrop((512, 512))
        print('AOI size is: ' + str(camera.getImageShape()))
        print('The readout time is: ' + str(camera.getReadoutTime()))
        print('The bytesyze required is: ' + str(camera.ImageSizeBytes.getValue()))
#    print('Horizontal Binning is: ' + str(camera.getHorizBin()))
    print('Sensor temp is:' + str(camera.getSensorTemperature()))
#     print('Temperature status is: ' + camera.getTemperatureStatus())
#     print('Setting temperature to -10')
    print('The camera handle is: ' + str(camera.handle))
    # We are now creating a MemoryHandler instance ouitside of the camera and passing the handle to test it
    mh = memoryHandler.MemoryHandler(camera.handle)
    print('This is the MemoryHandler instance: ' + str(mh))
    print('We are now allocating memory')
    error = mh.allocMemory(10, 409600, 512, 512, 0, 10)
    print(error)
    del mh



