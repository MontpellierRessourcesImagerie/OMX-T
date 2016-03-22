#!/usr/bin/python

###############
# AndorNeo.py
#
# Copyright David Baddeley, 2012
# d.baddeley@auckland.ac.nz
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
################


from AndorSDK3Camera import *
import numpy as np
import threading
import Queue
import Pyro4
import time
import traceback
import gc

import memoryHandler
# import ctypes
# import os


## Some configuration parameters

# Needed to keep the daemon from only listening to requests originating from the local host.
MY_IP_ADDRESS = '10.90.90.22'

# CameraNumber is by default 0, the first non sumulated camera
CAMERA_NUMBER = 0

# Memory we want to allocate for the camera buffers in Mb
MEMORY_ALLOCATION = 500

# Cropping modes
croppingModes = ('CROP_FULL',     # 0
                 'CROP_1024',     # 1
                 'CROP_512',      # 2
                 'CROP_256',      # 3
                 'CROP_128',      # 4
                 'CROP_ARBITRARY' # 5
                 )

croppingModesSizes = {'CROP_FULL': (2048,2048), # TODO: implement this depending on camera model
                      'CROP_1024': (1024,1024),
                      'CROP_512': (512,512),
                      'CROP_256': (256,256),
                      'CROP_128': (128,128),
                      'CROP_ARBITRARY': (2048,2048)
                      }

# Binning modes
BINNING_MODES = (u'1x1', u'2x2', u'3x3', u'4x4', u'8x8')
BINNING_VALUES = (1, 2, 3, 4, 8)

# Trigger modes
(TRIGGER_INTERNAL, TRIGGER_SOFTWARE, TRIGGER_EXTERNAL, TRIGGER_EXTERNAL_START, TRIGGER_EXTERNAL_EXPOSURE) = range(5)

# A black image
STATIC_BLACK = np.zeros((512, 512), dtype = np.uint16) # TODO: verify this is used


## Some helper classes

class DataThread(threading.Thread):
    '''
    This class retrieves images from the camera, and sends them to our client.
    '''
    def __init__(self, parent, width, height):
        threading.Thread.__init__(self)

        ## Loop back to parent to be able to communicate with it.
        self.parent = parent

        ## Image dimensions, which we need for when we retrieve image
        # data. Our parent is responsible for updating these for us.
        self.width = self.height = 0
        ## Lock on modifying the above.
        self.sensorLock = threading.Lock()

        ## Connection to client
        self.clientConnection = None

        ## Whether or not we should unload images from the camera
        self.shouldSendImages = True

        ## Initial timestamp that we will use in conjunction with time.clock()
        # to generate high-time-resolution timestamps. Just using time.time()
        # straight-up on Windows only has accuracy of ~15ms.
        self.initialTimestamp = time.time() + time.clock()

        ## Offset image array to subtract off of each image we receive.
        self.offsetImage = None


    ## Pull images from self.imageQueue and send them to the client.
    def run(self):
        count = 0
        gTime = None
        getTime = 0
        fixTime = 0
        sendTime = 0
        while True:
            # This will block indefinitely until images are available.
            with self.sensorLock:
                try:
                    start = time.clock()
                    image = self.parent.getImage()
                    getTime += (time.clock() - start)
                except Exception, e:
                    if 'getImage failed' not in e:
                        print "Error in getImage:",e
                    # Probably a timeout; just try again.
                    continue
            # \todo This timestamp is potentially bogus if we get behind in
            # processing images.
            timestamp = time.clock() + self.initialTimestamp
            if image:
                print "Image has shape",image.shape,"min/max",image.min(),image.max()
            start = time.clock()
            image = self.fixImage(image)
            fixTime += time.clock() - start
            count += 1
            if count % 100 == 0:
                # Periodically manually invoke the garbage collector, to
                # ensure that we don't build up a giant pile of work that
                # would interfere with our average write speed.
                if gTime is None:
                    gTime = time.time()
                delta = time.time() - gTime
                print count, delta, getTime, fixTime, sendTime
                gTime = time.time()
                getTime = fixTime = sendTime = 0
                gc.collect()

            if self.shouldSendImages and self.clientConnection is not None:
                try:
                    start = time.clock()
                    self.clientConnection.receiveData('new image', image, timestamp)
                    cost = time.clock() - start
                    if cost > .5:
                        print "Took %.2fs to send to client" % cost
                    sendTime += cost
                except Exception, e:
                    print "Failed to send image to client: %s", e
                    traceback.print_exc()


    ## Fix an image -- set its shape and apply any relevant correction.
    def fixImage(self, image):
        # image.shape = self.height, self.width
        if self.offsetImage is not None and self.offsetImage.shape == image.shape:
            # Apply offset correction.
            image -= self.offsetImage
        return image


    ## Update who we send image data to.
    def setClient(self, connection):
        self.clientConnection = connection


    ## Update our image dimensions.
    def setImageDimensions(self, width, height):
        with self.sensorLock:
            self.width = width
            self.height = height


    ## Update the image we use for offset correction.
    def setOffsetCorrection(self, image):
        self.offsetImage = image


    ## Retrieve our offset correction image.
    def getOffsetCorrection(self):
        return self.offsetImage

class AndorBase(SDK3Camera):
    numpy_frames=1
    MODE_CONTINUOUS = 1
    MODE_SINGLE_SHOT = 0

    def __init__(self, camNum):
        # define properties
        self.CameraAcquiring = ATBool() # Returns whether or not an acquisition is currently acquiring.
        self.SensorCooling = ATBool() # Configures the state of the sensor cooling. Cooling is disabled by default at power up and must be enabled for the camera to achieve its target temperature. The actual target temperature can be set with the TemperatureControl feature where available.

        self.AcquisitionStart = ATCommand() # Starts an acquisition
        self.AcquisitionStop = ATCommand() # Stops an acquisition

        self.CameraPresent = ATBool() # Returns whether the camera is connected to the system. Register a callback to this feature to be notified if the camera is disconnected. Notification of disconnection will not occur if CameraAcquiring is true, in this case AT_WaitBuffer will return an error.

        self.CycleMode = ATEnum() # Configures whether the camera will acquire a fixed length sequence or a continuous sequence. In Fixed mode the camera will acquire FrameCount number of images and then stop automatically. In Continuous mode the camera will continue to acquire images indefinitely until the AcquisitionStop command is issued.
        self.ElectronicShutteringMode = ATEnum() # Configures which on-sensor electronic shuttering mode is used
        self.FanSpeed = ATEnum() # Configures the speed of the fan in the camera
        self.PixelEncoding = ATEnum() # Configures the format of data stream.
        self.PixelReadoutRate = ATEnum() # Configures the rate of pixel readout from the sensor.
        self.TriggerMode = ATEnum() # Allows the user to configure the camera trigger mode at a high level. If the trigger mode is set to Advanced then the Trigger Selector and Trigger Source feature must also be set.

        self.AOIHeight = ATInt() # Configures the Height of the sensor area of interest in super-pixels.
        self.AOILeft = ATInt() # Configures the left hand coordinate of the sensor area of interest in sensor pixels.
        self.AOITop = ATInt() # Configures the top coordinate of the sensor area of interest in sensor pixels.
        self.AOIWidth = ATInt() # Configures the Width of the sensor area of interest in super-pixels.
        self.PixelHeight = ATFloat() # Returns the height of each pixel in micrometers.
        self.PixelWidth = ATFloat() # Returns the width of each pixel in micrometers.

        self.AOIHBin = ATInt() # Configures the Horizontal Binning of the sensor area of interest.
        self.AOIVBin = ATInt() # Configures the Vertical Binning of the sensor area of interest.
        self.AOIBinning = ATEnum() # Sets up pixel binning on the camera. Options: 1x1, 2x2, 3x3, 4x4, 8x8

        self.FrameCount = ATInt() # Configures the number of images to acquire in the sequence. The value of FrameCount must be any value which is a multiple of AccumulateCount. This ensures the accumulation contains the correct number of frames. When this feature is unavailable then the camera does not currently support fixed length series, therefore you must explicitly abort the acquisition once you have acquired the amount of frames required.
        self.ImageSizeBytes = ATInt() # Returns the buffer size in bytes required to store the data for one frame. This will be affected by the Area of Interest size, binning and whether metadata is appended to the data stream.
        self.SensorHeight = ATInt() # Returns the height of the sensor in pixels.
        self.SensorWidth = ATInt() # Returns the width of the sensor in pixels.

        self.CameraModel = ATString() # Returns the camera model
        self.SerialNumber = ATString() # Returns the camera serial number

        self.ExposureTime = ATFloat() # The requested exposure time in seconds. Note: In some modes the exposure time can also be modified while the acquisition is running
        self.FrameRate = ATFloat() # Configures the frame rate in Hz at which each image is acquired during any acquisition sequence. This is the rate at which frames are acquired by the camera which may be different from the rate at which frames are delivered to the user. For example when AccumulateCount has a value other than 1, the apparent frame rate will decrease proportionally.
        self.SensorTemperature = ATFloat() # Read the current temperature of the sensor.

        SDK3Camera.__init__(self, camNum)

        # Set the cropMode
        self.curCropMode = 0

        #end auto properties

#         self.camLock = threading.Lock()
#
#         self.buffersToQueue = Queue.Queue()
#         self.queuedBuffers = Queue.Queue()
#         self.fullBuffers = Queue.Queue()
#
#         self.nQueued = 0
#         self.nFull = 0
#
#         self.nBuffers = 100
#         self.defBuffers = 100
#
#
#         self.contMode = True
#         self.burstMode = False
#
#         self._temp = 0
#         self._frameRate = 0
#
#         self.active = True
#         #register as a provider of metadata
#         MetaDataHandler.provideStartMetadata.append(self.GenStartMetadata)

    def Init(self):
        SDK3Camera.Init(self)

        # cache some properties that we have to access regularly.

        self.CCDWidth = self.SensorWidth.getValue()
        self.CCDHeight = self.SensorHeight.getValue()

        self.width = self.AOIWidth.getValue()
        self.height = self.AOIHeight.getValue()

        self.SensorCooling.setValue(True)
#         self.setCropMode(2)


        # Print some of the camera infos

        print('Camera serial number: ' + self.getSerialNumber())
        print('CCD sensor shape: ' + str(self.width) + 'x' + str(self.height))
        print('Pixel encoding: ' + self.getPixelEncoding())
        print('Shutter mode: ' + str(self.getElectronicShutteringMode()))
        print('Fan speed: ' + self.FanSpeed.getString())
        print('Sensor cooling: ' + str(self.SensorCooling.getValue()))
        print('Sensor temperature: ' + str(self.SensorTemperature.getValue()))

        # Create a memoryHandler instance to manage the camera buffers
        self.memoryHandler = memoryHandler.MemoryHandler(self.handle)

        # Create a DataThread instance to manage the images transfer to the client
        self.dataThread = DataThread(self, self.width, self.height)

        self.dataThread.start()

    ## Some methods to manage memory
    def allocMemory(self, timeout = .5):
        '''
        Allocate memory to store images
        '''
        strides = self.getStrides()
        imageBytes = self.ImageSizeBytes.getValue()
        # We allocate about 500MB of RAM to the image buffer. Allocating
        # too much memory seems to cause slowdowns and crashes, oddly enough.
        numBuffers = (MEMORY_ALLOCATION * 1024 * 1024) / imageBytes
        self.memoryHandler.allocMemory(numBuffers,
                                       imageBytes,
                                       self.width,
                                       self.height,
                                       strides,
                                       timeout
                                       )

    def getStrides(self):
        '''
        Returns the strides on the images array.
        This is the bahaviour for a default SimCam and must be overridden in a
        hardware camera.
        '''
        return self.width * 16 # TODO: Not sure about this


    ## Some methods to manage data transfer
    def receiveClient(self, uri):
        '''
        Get told who we should be sending image data to, and how we
        should mark ourselves when we do.
        '''
        print('Receiving new client ' + str(uri))
        if uri is None:
            self.dataThread.setClient(None)
            self.dataThread.shouldSendImages = False
        else:
            connection = Pyro4.Proxy(uri)
            connection._pyroTimeout = 5
            self.dataThread.setClient(connection)
            self.dataThread.shouldSendImages = True

    def goSilent(self):
        '''
        Stop sending images to the client, even if we still
        receive them.
        '''
        print "Switching to quiet mode"
        self.dataThread.shouldSendImages = False

    def goLoud(self):
        '''
        Start sending new images to the client again.
        '''
        print "Switching to loud mode"
        self.dataThread.shouldSendImages = True
        
    def getImage(self, timeout = .5):
        '''
        get a single image wrapped around the memoryHandler getImage
        '''
        return self.memoryHandler.getImage(timeout)
    

    def getImages(self, numImages, timeout = .5):
        '''
        Retrieve the specified number of images. This may fail
        if we ask for more images than we have, so the caller
        should be prepared to catch exceptions.
        '''
        result = []
        for i in xrange(numImages):
            image = self.getImage(timeout)
            result.append(image)
        return result

    def discardImages(self, numImages = None):
        '''
        Discard the specified number of images from the queue,
        defaulting to all of them.
        '''
        count = 0
        while count != numImages:
            try:
                self.getImage(0)
                count += 1
            except Exception, e:
                # Got an exception, probably because the
                # queue is empty, so we're all done here.
                return


        #set some initial parameters
        #self.FrameCount.setValue(1)
#         self.CycleMode.setString(u'Continuous')
#
#         set up polling thread
#         self.doPoll = False
#         self.pollLoopActive = True
#         self.pollThread = threading.Thread(target = self._pollLoop)
#         self.pollThread.start()
#


    #Neo buffer helper functions

#     def InitBuffers(self):
#         self._flush()
#         bufSize = self.ImageSizeBytes.getValue()
#         vRed = int(self.SensorHeight.getValue()/self.AOIHeight.getValue())
#         self.nBuffers = vRed*self.defBuffers
#         #print bufSize
#         for i in range(self.nBuffers):
#             #buf = np.empty(bufSize, 'uint8')
#             buf = create_aligned_array(bufSize, 'uint8')
#             self._queueBuffer(buf)
#
#         self.doPoll = True

#     def _flush(self):
#         self.doPoll = False
#         #purge camera buffers
#         SDK3.Flush(self.handle)
#
#         #purge our local queues
#         while not self.queuedBuffers.empty():
#             self.queuedBuffers.get()
#
#         while not self.buffersToQueue.empty():
#             self.buffersToQueue.get()
#
#         self.nQueued = 0
#
#         while not self.fullBuffers.empty():
#             self.fullBuffers.get()
#
#         self.nFull = 0
#         #purge camera buffers
#         SDK3.Flush(self.handle)

#     def _queueBuffer(self, buf):
#         #self.queuedBuffers.put(buf)
#         #print np.base_repr(buf.ctypes.data, 16)
#         #SDK3.QueueBuffer(self.handle, buf.ctypes.data_as(SDK3.POINTER(SDK3.AT_U8)), buf.nbytes)
#         #self.nQueued += 1
#         self.buffersToQueue.put(buf)

#     def _queueBuffers(self):
#         #self.camLock.acquire()
#         while not self.buffersToQueue.empty():
#             buf = self.buffersToQueue.get(block=False)
#             self.queuedBuffers.put(buf)
#             #print np.base_repr(buf.ctypes.data, 16)
#             SDK3.QueueBuffer(self.handle, buf.ctypes.data_as(SDK3.POINTER(SDK3.AT_U8)), buf.nbytes)
#             #self.fLog.write('%f\tq\n' % time.time())
#             self.nQueued += 1
#         #self.camLock.release()

#     def _pollBuffer(self):
#         try:
#             #self.fLog.write('%f\tp\n' % time.time())
#             pData, lData = SDK3.WaitBuffer(self.handle, 100)
#             #self.fLog.write('%f\tb\n' % time.time())
#         except SDK3.TimeoutError as e:
#             #Both AT_ERR_TIMEDOUT and AT_ERR_NODATA
#             #get caught as TimeoutErrors
#             #if e.errNo == SDK3.AT_ERR_TIMEDOUT:
#             #    self.fLog.write('%f\tt\n' % time.time())
#             #else:
#             #    self.fLog.write('%f\tn\n' % time.time())
#             return
#         except SDK3.CameraError as e:
#             if not e.errNo == SDK3.AT_ERR_NODATA:
#                 traceback.print_exc()
#             return
#
#         #self.camLock.acquire()
#         buf = self.queuedBuffers.get()
#         self.nQueued -= 1
#         if not buf.ctypes.data == ctypes.addressof(pData.contents):
#             print((ctypes.addressof(pData.contents), buf.ctypes.data))
#             #self.camLock.release()
#             raise RuntimeError('Returned buffer not equal to expected buffer')
#             #print 'Returned buffer not equal to expected buffer'
#
#         self.fullBuffers.put(buf)
#         self.nFull += 1
#         #self.camLock.release()

#     def _pollLoop(self):
#         #self.fLog = open('poll.txt', 'w')
#         while self.pollLoopActive:
#             self._queueBuffers()
#             if self.doPoll: #only poll if an acquisition is running
#                 self._pollBuffer()
#             else:
#                 #print 'w',
#                 time.sleep(.05)
#             time.sleep(.0005)
#             #self.fLog.flush()
#         #self.fLog.close()

#     #PYME Camera interface functions - make this look like the other cameras
#     def ExpReady(self):
#         #self._pollBuffer()
#
#         return not self.fullBuffers.empty()

#     def ExtractColor(self, chSlice, mode):
#         #grab our buffer from the full buffers list
#         buf = self.fullBuffers.get()
#         self.nFull -= 1
#
#         #copy to the current 'active frame'
#         #print chSlice.shape, buf.view(chSlice.dtype).shape
#         #bv = buf.view(chSlice.dtype).reshape(chSlice.shape)
#         xs, ys = chSlice.shape[:2]
#
#         a_s = self.AOIStride.getValue()
#
#         #print buf.nbytes
#         #bv = buf.view(chSlice.dtype).reshape([-1, ys], order='F')
#
# #        bv = np.ndarray(shape=[xs,ys], dtype='uint16', strides=[2, a_s], buffer=buf)
# #        chSlice[:] = bv
#
#         #chSlice[:,:] = bv
#         #ctypes.cdll.msvcrt.memcpy(chSlice.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), buf.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), chSlice.nbytes)
#         #ctypes.cdll.msvcrt.memcpy(chSlice.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), buf.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), chSlice.nbytes)
#         #print 'f'
#
#         dt = self.PixelEncoding.getString()
#
#         SDK3.ConvertBuffer(buf.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), chSlice.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), xs, ys, a_s, dt, 'Mono16')
#
#         #recycle buffer
#         self._queueBuffer(buf)

    def resetCam(self):
        '''
        Resets the camera
        '''
        pass # TODO: implement this

    def setTrigger(self, triggerMode):
        '''
        Changes the triggering mode of the camera
        '''
        self.TriggerMode.setIndex(triggerMode)

        # Start the acquisition with external triggers
        if 1 < triggerMode:
            self.AcquisitionStart()

    def getTrigger(self):
        '''
        Returns the triggering mode of the camera
        '''
        return self.TriggerMode.getIndex()

    def setElectronicShutteringMode(self, isGlobal):
        '''
        Changes the shutter mode.0 is rolling shutter; 1 is global
        '''
        self.ElectronicShutteringMode.setIndex(isGlobal)

    def getElectronicShutteringMode(self):
        '''
        Get the current shutter mode. 0 is rolling shutter; 1 is global
        '''
        return self.ElectronicShutteringMode.getIndex()

    def setExposureTime(self, time):
        '''
        Changes the exposure time in the camera. In seconds
        '''
        self.ExposureTime.setValue(time) # TODO: Impleent with a reset?

    def getExposureTime(self):
        '''
        Returns the exposure time of the camera. In seconds
        '''
        return self.ExposureTime.getValue()

    def getMinExposureTime(self):
        '''
        Returns the minimum exposure time accepted by the camera. In seconds
        '''
        return self.ExposureTime.min()

    def setCropMode(self, mode):
        '''
        Previously setCrop. Set the cropping mode to one of a few presets.
        In version 2 of the SDK, these were the only valid crop modes; in
        version 3, we can set any crop mode we want, but these
        presets are still convenient.
        mode is an integer.
        '''
        self.curCropMode = mode # TODO: implement arbitrary cropMode
        self.setCrop(croppingModesSizes[croppingModes[mode]])

    def setCrop(self, cropSize, binning = 0):
        '''
        Changes the AOI in the camera.

        cropSize is a tupple or list of two integers providing the size of the AOI (x, y).
        binning must be a string
        AOI will be centered in the camera
        '''
        # cropSize must be converted into superpixel size in case there is binning
        binnedCropSize = cropSize
        if binning != 0:
            binnedCropSize[0] = cropSize[0] // BINNING_VALUES[binning]
            binnedCropSize[1] = cropSize[1] // BINNING_VALUES[binning]
            
        self.setCropArbitrary(binnedCropSize[0],
                              ((self.CCDWidth - cropSize[0]) // 2) + 1,
                              binnedCropSize[1],
                              ((self.CCDHeight - cropSize[1]) // 2) + 1,
                              binning,
                              )

    def setCropArbitrary(self, width, left, height, top, binning = u'1x1'):

        # Set the binning
        self.AOIBinning.setIndex(binning)
  
        # Set AOI
        self.AOIWidth.setValue(width)
        self.AOILeft.setValue(left)
        self.AOIHeight.setValue(height)
        self.AOITop.setValue(top)

        # update width and height values
        self.width = self.AOIWidth.getValue()
        self.height = self.AOIHeight.getValue()
        
        # reallocate memory
        self.allocMemory()

    def getCropMode(self):
        return self.curCropMode

    def getImageShape(self):
        '''
        Returns the image size (AOI) as a tupple of two integers (x, y)
        '''
        return (self.width, self.height)

    def setOffsetCorrection(self, image):
        '''
        Set an offset correction image to use.
        '''
        self.dataThread.setOffsetCorrection(image)

    def getIsOffsetCorrectionOn(self):
        '''
        Return true if a correction file is loaded for the current image dimensions.
        '''
        correction = self.dataThread.getOffsetCorrection()
        return correction is not None and correction.shape == (self.height, self.width)

    def getSerialNumber(self):
        return self.SerialNumber.getValue()

    def getCameraModel(self):
        return self.CameraModel.getValue()

    def setIntegTime(self, iTime):
        self.ExposureTime.setValue(iTime*1e-3)
        self.FrameRate.setValue(self.FrameRate.max())

    def getIntegTime(self):
        return self.ExposureTime.getValue()

    def getCycleTime(self):
        return 1.0/self.FrameRate.getValue()

    def setPixelReadoutRate(self, rate):
        '''
        Sets the PixelReadoutRate.
        rete must be a string. For sCMOS: '280 MHz' or '100 MHz'.
        For the SimCam: '550 MHz'
        '''
        self.PixelReadoutRate.setString(rate)

    def getPixelEncoding(self):
        return self.PixelEncoding.getString()

    def getCCDWidth(self):
        return self.CCDWidth

    def getCCDHeight(self):
        return self.CCDHeight

    def setHorizBin(self, horizontalBinning):
        self.AOIHBin.setValue(horizontalBinning)

    def getHorizBin(self):
        return self.AOIHBin.getValue()

    def setVertBin(self, verticalBinning):
        self.AOIVBin.setValue(verticalBinning)

    def getVertBin(self):
        return self.AOIVBin.getValue()

    def getSensorTemperature(self):
        # for some reason querying the temperature takes a lot of time - do it less often
        return self.SensorTemperature.getValue()

    def getTemperatureStatus(self):
        # returns the status of the temperature sensor
        return self.TemperatureStatus.getString()

    def isReady(self):
        return True

    def getAOIWidth(self):
        return self.width

    def getAOIHeight(self):
        return self.height

    def abort(self):
        '''
        Stop acquiring Images
        '''
        if self.CameraAcquiring.getValue():
            self.AcquisitionStop()

    def Shutdown(self):
        print 'Shutting down sCMOS camera'
        self.pollLoopActive = False ## TODO: remove this
        self.shutdown()

#     def GetStatus(*args):
#         pass

#     def SetCOC(*args):
#         pass

#     def StartExposure(self):
#         #make sure no acquisiton is running
#         self.StopAq()
#         self._temp = self.SensorTemperature.getValue()
#         self._frameRate = self.FrameRate.getValue()
#
#         eventLog.logEvent('StartAq', '')
#         self._flush()
#         self.InitBuffers()
#         self.AcquisitionStart()
#
#         return 0

#     def StopAq(self):
#         if self.CameraAcquiring.getValue():
#             self.AcquisitionStop()

#     #new fcns for Andor compatibility
#     def GetNumImsBuffered(self):
#         return self.nFull

#     def GetBufferSize(self):
#         return self.nBuffers

#     def SetActive(self, active=True):
#         '''flag the camera as active (or inactive) to dictate whether it writes it's metadata or not'''
#         self.active = active

#     def GenStartMetadata(self, mdh):
#         if self.active:
#             self.GetStatus()
#
#             mdh.setEntry('Camera.Name', 'Andor Zyla')
#
#             mdh.setEntry('Camera.IntegrationTime', self.GetIntegTime())
#             mdh.setEntry('Camera.CycleTime', self.GetCycleTime())
#             mdh.setEntry('Camera.EMGain', 1)
#
#             mdh.setEntry('Camera.ROIPosX', self.GetROIX1())
#             mdh.setEntry('Camera.ROIPosY',  self.GetROIY1())
#             mdh.setEntry('Camera.ROIWidth', self.GetROIX2() - self.GetROIX1())
#             mdh.setEntry('Camera.ROIHeight',  self.GetROIY2() - self.GetROIY1())
#             #mdh.setEntry('Camera.StartCCDTemp',  self.GetCCDTemp())
#
#             mdh.setEntry('Camera.ReadNoise', 1)
#             mdh.setEntry('Camera.NoiseFactor', 1)
#             mdh.setEntry('Camera.ElectronsPerCount', .28)
#             mdh.setEntry('Camera.ADOffset', self.Baseline.getValue())
#
#             #mdh.setEntry('Simulation.Fluorophores', self.fluors.fl)
#             #mdh.setEntry('Simulation.LaserPowers', self.laserPowers)
#
#             #realEMGain = ccdCalibrator.CalibratedCCDGain(self.GetEMGain(), self.GetCCDTempSetPoint())
#             #if not realEMGain == None:
#             mdh.setEntry('Camera.TrueEMGain', 1)
#
#             itime = int(1000*self.GetIntegTime())
#             calpath = nameUtils.getCalibrationDir(self.GetSerialNumber())
#             dkfn = os.path.join(calpath, 'dark_%dms.tif'%itime)
#             print dkfn
#             if os.path.exists(dkfn):
#                 mdh['Camera.DarkMapID'] = dkfn
#             varfn = os.path.join(calpath, 'variance_%dms.tif'%itime)
#             print varfn
#             if os.path.exists(varfn):
#                 mdh['Camera.VarianceMapID'] = varfn

#     #functions to make us look more like andor camera
#     def GetEMGain(self):
#         return 1

#     def GetCCDTempSetPoint(self):
#         return self.TargetSensorTemperature.getValue()

#     def SetCCDTemp(self, temp):
#         self.TargetSensorTemperature.setValue(temp)
#         #pass

#     def SetEMGain(self, gain):
#         pass

#     def SetAcquisitionMode(self, aqMode):
#         self.CycleMode.setIndex(aqMode)
#         self.contMode = aqMode == self.MODE_CONTINUOUS

#     def SetBurst(self, burstSize):
#         if burstSize > 1:
#             self.SetAcquisitionMode(self.MODE_SINGLE_SHOT)
#             self.FrameCount.setValue(burstSize)
#             self.contMode = True
#             self.burstMode = True
#         else:
#             self.FrameCount.setValue(1)
#             self.SetAcquisitionMode(self.MODE_CONTINUOUS)
#             self.burstMode = False

#     def SetShutter(self, mode):
#         pass

#     def SetBaselineClamp(self, mode):
#         pass

#     def GetFPS(self):
#         #return self.FrameRate.getValue()
#         return self._frameRate

    def __del__(self):
        self.Shutdown()
        #self.compT.kill = True


class AndorZyla(AndorBase):
    def __init__(self, camNum):
        #define properties

        self.AOIStride = ATInt() # The size of one row in the image in bytes. Extra padding bytes may be added to the end of each line after pixel data to comply with line size granularity restrictions imposed by the underlying hardware interface.

        self.Baseline = ATInt() # Returns the baseline level of the image with current settings

        self.CameraName = ATString() # Returns the name of the camera.

        self.Overlap = ATBool() # Enables overlap readout mode.
        self.SpuriousNoiseFilter = ATBool() # Enables or Disables the Spurious Noise Filter
        self.StaticBlemishCorrection = ATBool() # Enables or Disables Static Blemish Correction

        self.VerticallyCentreAOI = ATBool() # Vertically centres the AOI in the frame. With this enabled, AOITop will be disabled.

        self.CameraDump = ATCommand() # Dumps current hardware configuration information to file in the executable directory. File is called camdump-Serial Number
        self.SoftwareTrigger = ATCommand() # Generates a software trigger in the camera. Used to generate each frame on the camera whenever the trigger mode is set to Software.

        self.ExternalTriggerDelay = ATFloat() # Sets the delay time between the camera receiving an external trigger and the acquisition start.
        self.FastAOIFrameRateEnable = ATBool() # Enables faster framerates at small AOIs.
        self.RollingShutterGlobalClear = ATBool() # Enables Rolling Shutter Global Clear readout mode.
        self.RowReadTime = ATFloat() # Configures the time in seconds to read a single row.
        self.SensorReadoutMode = ATEnum() # Configures the direction in which the sensor will be read out
        self.ShutterOutputMode = ATEnum() # Controls the mode the external trigger will run in. External Shutter signal can either be set to high (open) or low (closed). ShutterOutput can be triggered by setting AuxOutSourceTwo to ExternalShutterControl

        self.TemperatureStatus = ATEnum() # Reports the current state of cooling towards the Target Sensor Temperature. Read Only
        self.SimplePreAmpGainControl = ATEnum() # Wrapper Feature to simplify selection of the sensitivity and dynamic range options. This feature should be used as a replacement for the PreAmpGainControl feature as some of the options in the PreAmpGainControl feature are not supported on all cameras. Supported Bit Depth will be dependent on the camera
        self.BitDepth = ATEnum() # Returns the number bits used to store information about each pixel of the image. Supported Bit Depth will be dependent on the camera.
        self.MetadataEnable = ATBool() # Enable metadata. This is a global flag which will enable inclusion of metadata in the data stream as described in Section 4.5 Metadata. When this flag is enabled the data stream will always contain the MetadataFrame information. This will override the subsequent metadata settings when disabled.
        self.MetadataFrame = ATBool() # Indicates whether the MetadataFrame information is included in the data stream. This is read only and is automatically sent if metadata is enabled.
        self.MetadataTimestamp = ATBool() # Enables inclusion of timestamp information in the metadata stream. The timestamp indicates the time at which the exposure for the frame started.

#         self.ActualExposureTime = ATFloat()
#         self.BurstRate = ATFloat()
        self.ReadoutTime = ATFloat() # This feature will return the time to readout data from a sensor.
        self.ExposedPixelHeight = ATInt() # Configures the exposure window in pixels.

        self.TimestampClock = ATInt() # Reports the current value of the camera internal timestamp clock. This same clock is used to timestamp images as they are acquired when the MetadataTimestamp feature is enabled. The clock is reset to zero when the camera is powered on and then runs continuously at the frequency indicated by the TimestampClockFrequency feature. The clock is 64-bits wide.
        self.TimestampClockFrequency = ATInt() # Reports the frequency of the camera internal timestamp clock in Hz.
        self.TimestampClockReset = ATCommand() # Resets the camera internal timestamp clock to zero. As soon as the reset is complete the clock will begin incrementing from zero at the rate given by the TimestampClockFrequency feature.

        self.AccumulateCount = ATInt() # Sets the number of images that should be summed to obtain each image in sequence.
        self.Baseline = ATInt() # Returns the baseline level of the image with current settings
#         self.BurstCount = ATInt()
        self.LUTIndex = ATInt() # Sets the position in the LUT to read/write a new pixel map
        self.LUTValue = ATInt() # Sets the value in LUT in position specified by LUT Index

        self.ControllerID = ATString() # Returns a unique identifier for the camera controller device. i.e. Frame grabber over Cameralink
        self.FirmwareVersion = ATString() # Returns the camera firmware version


        AndorBase.__init__(self,camNum)

    def Init(self):
        AndorBase.Init(self)

        print('Bit depth: ' + self.BitDepth.getString())
        print('Temperature status: ' + self.getTemperatureStatus())
        print('FirmwareVersion: ' + self.FirmwareVersion.getValue())
        print('Baseline level: ' + str(self.Baseline.getValue()))

        # Configure default camera status

        self.setFanSpeed(u'Off')
        self.setSimplePreAmpGainControl(u'16-bit (low noise & high well capacity)')
        self.setPixelReadoutRate(u'280 MHz')


    # Define Zyla specific methods
    
    def getAccumulateCount(self):
        return self.AccumulateCount.getValue()
    
    def setAccumulateCount(self, nrOfFrames = 1):
        self.AccumulateCount.setValue(nrOfFrames)

    def getReadoutTime(self):
        '''
        Returns the readout time in seconds as a float
        '''
        return self.ReadoutTime.getValue()

    def getStrides(self):
        '''
        Returns the strides on the images array.
        This is the bahaviour for a default SimCam and must be overridden in a
        hardware camera.
        '''
        return self.AOIStride.getValue() # TODO: Not sure about this

    def setFanSpeed(self, speed):
        '''
        Sets the fan speed. For Zyla speed is 'On' or 'Off'
        '''
        self.FanSpeed.setString(speed)

    def setSimplePreAmpGainControl(self, stringValue):
        '''
        Sets the sensitivity and dynamic range options
        stringValue is:
        11-bit (high well capacity)
        12-bit (high well capacity)
        11-bit (low noise)
        12-bit (low noise)
        16-bit (low noise & high well capacity)
        '''
        self.SimplePreAmpGainControl.setString(stringValue)


class AndorSim(AndorBase):
    def __init__(self, camNum):
        #define properties
        self.SynchronousTriggering = ATBool() # Configures whether external triggers are synchronous with the read out of a sensor row. Asynchronous triggering may result in data corruption in the row being digitised when the triggers occurs.

        self.PixelCorrection = ATEnum() # Configures the pixel correction to be applied.
        self.TriggerSelector = ATEnum() # Only if trigger mode in advanced
        self.TriggerSource = ATEnum()  # Only if trigger mode in advanced


        AndorBase.__init__(self,camNum)

if __name__ == '__main__':
    try:
        cam = AndorZyla(camNum = CAMERA_NUMBER)
        daemon = Pyro4.Daemon(port = 7000, host = MY_IP_ADDRESS)
        Pyro4.Daemon.serveSimple(
            {
                cam: 'Andorcam',
            },
            daemon = daemon, ns = False, verbose = True
        )
    
    except Exception, e:
        traceback.print_exc()
    
    # Clean up after ourselves.
    # TODO: add some del's to clean up
