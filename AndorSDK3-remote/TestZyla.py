'''
A very basic module to test the Zyla camera
'''
from AndorZyla import AndorBase, AndorZyla
from __builtin__ import str
#import memoryHandler

print('Importing cameras...')

from AndorSDK3Camera import *

print('Cameras Imported')

numCameras = GetNumCameras()
print(str(numCameras) + ' cameras were detected')

softwareVersion = GetSoftwareVersion().value
print('Software version is: ' + softwareVersion)

camerasList = []

if numCameras == 3:
    camerasList.append(AndorZyla(0))
else:
    for i in range(numCameras):
        camerasList.append(AndorBase(i))



for camera in camerasList:
    print('Initialising camera...')
    camera.Init()
    print('Camera model: ' + camera.getCameraModel())
    print('Camera serial nr: ' + camera.getSerialNumber())
    print('Current trigger mode: ' + str(camera.getTrigger()))
#     print('Changing trigger mode to 0...')
#     camera.setTrigger(0)
    print('Current trigger mode: ' + camera.TriggerMode.getString())
    print('Available triggerModes:')
    modes = camera.TriggerMode.getAvailableValues()
    print(type(modes))
#     print('Changing trigger mode back to Internal...')
#     camera.TriggerMode.setString(u'Internal')
    print('Current trigger mode: ' + str(camera.getTrigger()))
    print('Current exposure time: ' + str(camera.getExposureTime()))
    print('Changing exposure to 1...')
    camera.setExposureTime(1)
    print('Current exposure time: ' + str(camera.getExposureTime()))
    print('The minimum exposure time is: ' + str(camera.getMinExposureTime()))
    if camera.getCameraModel() != 'SIMCAM CMOS':
        print('AOI size is: ' + str(camera.AOIWidth.getValue()) + 'x' + str(camera.AOIHeight.getValue()))
        print('Changing AOI to 512x512...')
        camera.setCrop((512, 512))
        print('AOI size is: ' + str(camera.getImageShape()))
        print('The readout time is: ' + str(camera.getReadoutTime()))
        print('The bytesyze required is: ' + str(camera.ImageSizeBytes.getValue()))
    print('Sensor temp is:' + str(camera.getSensorTemperature()))
    camera.FrameCount.setValue(10)
#     camera.FrameRate.setValue(1)
    camera.setTrigger(0)
#     camera.AcquisitionStop()
#     camera.AcquisitionStart()

