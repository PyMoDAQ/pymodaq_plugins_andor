#!/usr/bin/python

###############
# andor_sdk3.py
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

# modified by S. Weber 2020
# PyMoDAQ compatibility
################

from . import sdk3cam
from . import sdk3
import numpy as np
import threading
import ctypes
import time
import traceback
import logging
import queue as Queue


class AndorBase(sdk3cam.SDK3Camera):
    numpy_frames = 1
    MODE_CONTINUOUS = 1
    MODE_SINGLE_SHOT = 0

    validROIS = [(2592, 2160, 1, 1),
                 (2544, 2160, 1, 25),
                 (2064, 2048, 57, 265),
                 (1776, 1760, 201, 409),
                 (1920, 1080, 537, 337),
                 (1392, 1040, 561, 601),
                 (528, 512, 825, 1033),
                 (240, 256, 953, 1177),
                 (144, 128, 1017, 1225)]

    def __init__(self, camNum, nbuffer=1):
        # define properties
        self.CameraAcquiring = sdk3cam.ATBool()
        self.SensorCooling = sdk3cam.ATBool()

        self.AcquisitionStart = sdk3cam.ATCommand()
        self.AcquisitionStop = sdk3cam.ATCommand()

        self.CycleMode = sdk3cam.ATEnum()
        self.ElectronicShutteringMode = sdk3cam.ATEnum()
        self.FanSpeed = sdk3cam.ATEnum()
        self.PreAmpGainChannel = sdk3cam.ATEnum()
        self.PixelEncoding = sdk3cam.ATEnum()
        self.PixelReadoutRate = sdk3cam.ATEnum()
        self.PreAmpGain = sdk3cam.ATEnum()
        self.PreAmpGainSelector = sdk3cam.ATEnum()
        self.TriggerMode = sdk3cam.ATEnum()

        self.FullAOIControl = sdk3cam.ATBool()
        self.AOIHeight = sdk3cam.ATInt()
        self.AOILeft = sdk3cam.ATInt()
        self.AOITop = sdk3cam.ATInt()
        self.AOIWidth = sdk3cam.ATInt()
        self.AOIStride = sdk3cam.ATInt()

        self.FrameCount = sdk3cam.ATInt()
        self.ImageSizeBytes = sdk3cam.ATInt()
        self.SensorHeight = sdk3cam.ATInt()
        self.SensorWidth = sdk3cam.ATInt()

        self.ShutterMode = sdk3cam.ATEnum()
        self.ShutterOutputMode = sdk3cam.ATEnum()

        self.CameraModel = sdk3cam.ATString()
        self.CameraName = sdk3cam.ATString()
        self.ControllerID = sdk3cam.ATString()
        self.SerialNumber = sdk3cam.ATString()

        self.ExposureTime = sdk3cam.ATFloat()
        self.FrameRate = sdk3cam.ATFloat()
        self.SensorTemperature = sdk3cam.ATFloat()
        self.TargetSensorTemperature = sdk3cam.ATFloat()

        sdk3cam.SDK3Camera.__init__(self, camNum)

        # end auto properties

        self.camLock = threading.Lock()

        self.buffersToQueue = Queue.Queue()
        self.queuedBuffers = Queue.Queue()
        self.fullBuffers = Queue.Queue()

        self.nQueued = 0
        self.nFull = 0

        self.nBuffers = 100

        self.contMode = True
        self.burstMode = False

        self._temp = 0
        self._frameRate = 0


    def init_camera(self, polling_enable=True):
        super().init_camera()
        self.polling_enable = polling_enable
        # set some intial parameters
        # self.FrameCount.setValue(1) #only for fixed mode?
        # if self.CycleMode.isImplemented():
        #     self.CycleMode.setString(u'Continuous')
        # # if self.SimplePreAmpGainControl.isImplemented():
        # #     self.SimplePreAmpGainControl.setString(u'11-bit (low noise)')
        # if self.PixelEncoding.isImplemented():
        #     self.PixelEncoding.setString('Mono12Packed')  # FIXME allow Mono16
        # if self.SensorCooling.isImplemented():
        #     self.SensorCooling.setValue(True)  # FIXME allow cooling selection
        # try:
        #     self.TemperatureControl.setString('-30.00')
        # except sdk3.CameraError:
        #     # Some camera cannot write the temperature control and returns
        #     # AT_ERR_NOTWRITABLE
        #     pass
        # self.PixelReadoutRate.setIndex(1)

        if self.polling_enable:
            # set up polling thread
            self.doPoll = False
            self.pollLoopActive = True
            self.pollThread = threading.Thread(target=self._pollLoop)
            self.pollThread.start()

    # Neo buffer helper functions

    def initBuffers(self):
        self._flush()
        bufSize = self.ImageSizeBytes.getValue()
        # print(bufSize)
        for i in range(self.nBuffers):
            buf = np.empty(bufSize, 'uint8')
            #buf = create_aligned_array(bufSize, 'uint8')
            self._queueBuffer(buf)

        self.doPoll = True

    def _flush(self):
        self.doPoll = False
        # purge camera buffers
        sdk3.Flush(self.handle)

        # purge our local queues
        while not self.queuedBuffers.empty():
            self.queuedBuffers.get()

        while not self.buffersToQueue.empty():
            self.buffersToQueue.get()

        self.nQueued = 0

        while not self.fullBuffers.empty():
            self.fullBuffers.get()

        self.nFull = 0
        # purge camera buffers
        sdk3.Flush(self.handle)

    def _queueBuffer(self, buf):
        # self.queuedBuffers.put(buf)
        # print np.base_repr(buf.ctypes.data, 16)
        #sdk3.QueueBuffer(self.handle, buf.ctypes.data_as(sdk3.POINTER(sdk3.AT_U8)), buf.nbytes)
        #self.nQueued += 1
        self.buffersToQueue.put(buf)

    def _queueBuffers(self):
        # self.camLock.acquire()
        while not self.buffersToQueue.empty():
            buf = self.buffersToQueue.get(block=False)
            self.queuedBuffers.put(buf)
            # print np.base_repr(buf.ctypes.data, 16)
            sdk3.QueueBuffer(self.handle, buf.ctypes.data_as(sdk3.POINTER(sdk3.AT_U8)), buf.nbytes)
            #self.fLog.write('%f\tq\n' % time.time())
            self.nQueued += 1
        # self.camLock.release()

    def queue_single_buffer(self, buf):
        sdk3.QueueBuffer(self.handle, buf.ctypes.data_as(sdk3.POINTER(sdk3.AT_U8)), buf.nbytes)

    def wait_buffer(self):
        try:
            pData, lData = sdk3.WaitBuffer(self.handle, 100)
        except sdk3.TimeoutError as e:
            return
        except sdk3.CameraError as e:
            #    if not e.errNo == sdk3.AT_ERR_NODATA:
            #        traceback.print_exc()
            return
        return ctypes.addressof(pData.contents)

    def get_image_fom_buffer(self, Nx, Ny, buffer):
        data = np.empty((Ny, Nx), np.uint16)

        a_s = self.AOIStride.getValue()
        dt = self.PixelEncoding.getString()
        sdk3.ConvertBuffer(buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
                           data.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
                           Nx, Ny, a_s, dt, 'Mono16')

        return data


    def flush(self):
        self._flush()

    def _pollBuffer(self):
        try:
            #self.fLog.write('%f\tp\n' % time.time())
            pData, lData = sdk3.WaitBuffer(self.handle, 100)
            #self.fLog.write('%f\tb\n' % time.time())
        except sdk3.TimeoutError as e:
            # Both AT_ERR_TIMEDOUT and AT_ERR_NODATA
            # get caught as TimeoutErrors
            # if e.errNo == sdk3.AT_ERR_TIMEDOUT:
            #    self.fLog.write('%f\tt\n' % time.time())
            # else:
            #    self.fLog.write('%f\tn\n' % time.time())
            return
        except sdk3.CameraError as e:
            #    if not e.errNo == sdk3.AT_ERR_NODATA:
            #        traceback.print_exc()
            return

        # self.camLock.acquire()
        buf = self.queuedBuffers.get()
        self.nQueued -= 1
        if not buf.ctypes.data == ctypes.addressof(pData.contents):
            print((ctypes.addressof(pData.contents), buf.ctypes.data))
            # self.camLock.release()
            raise RuntimeError('Returned buffer not equal to expected buffer')
            # print 'Returned buffer not equal to expected buffer'

        self.fullBuffers.put(buf)
        self.nFull += 1
        # self.camLock.release()

    def _pollLoop(self):
        #self.fLog = open('poll.txt', 'w')
        while self.pollLoopActive:
            self._queueBuffers()
            if self.doPoll:  # only poll if an acquisition is running
                self._pollBuffer()
            else:
                # print 'w',
                time.sleep(.05)
            time.sleep(.0005)
            # self.fLog.flush()
        # self.fLog.close()

    # PYME Camera interface functions - make this look like the other cameras
    def ExpReady(self):
        # self._pollBuffer()

        return not self.fullBuffers.empty()

    def ExtractColor(self, chSlice, mode):
        # grab our buffer from the full buffers list
        buf = self.fullBuffers.get()
        self.nFull -= 1

        # copy to the current 'active frame'
        # print chSlice.shape, buf.view(chSlice.dtype).shape
        #bv = buf.view(chSlice.dtype).reshape(chSlice.shape)
        #chSlice[:] = bv
        #chSlice[:,:] = bv

        # FIXME mscvrt windows only?
        #ctypes.cdll.msvcrt.memcpy(chSlice.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), buf.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)), chSlice.nbytes)

        xs, ys = chSlice.shape[:2]
        a_s = self.AOIStride.getValue()
        dt = self.PixelEncoding.getString()
        sdk3.ConvertBuffer(buf.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
                           chSlice.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
                           xs, ys, a_s, dt, 'Mono16')

        # recycle buffer
        self._queueBuffer(buf)

    def GetSerialNumber(self):
        return self.SerialNumber.getValue()

    def SetIntegTime(self, iTime):
        """Set Integration time in milliseconds"""
        self.ExposureTime.setValue(iTime * 1e-3)
        self.FrameRate.setValue(self.FrameRate.max())

    def GetIntegTime(self):
        """Return Integration Time in milliseconds"""
        return self.ExposureTime.getValue() * 1000

    def GetCCDWidth(self):
        return self.SensorWidth.getValue()

    def GetCCDHeight(self):
        return self.SensorHeight.getValue()

    def SetHorizBin(*args):
        raise Exception('Not implemented yet!!')

    def GetHorizBin(*args):
        return 0
        # raise Exception, 'Not implemented yet!!'

    def GetHorzBinValue(*args):
        raise Exception('Not implemented yet!!')

    def SetVertBin(*args):
        raise Exception('Not implemented yet!!')

    def GetVertBin(*args):
        return 0
        # raise Exception, 'Not implemented yet!!'

    def GetNumberChannels(*args):
        raise Exception('Not implemented yet!!')

    def GetElectrTemp(*args):
        return 25

    def GetCCDTemp(self):
        # for some reason querying the temperature takes a lot of time - do it less often
        # return self.SensorTemperature.getValue()

        return self._temp

    def CamReady(*args):
        return True

    def GetPicWidth(self):
        return self.AOIWidth.getValue()

    def GetPicHeight(self):

        return self.AOIHeight.getValue()

    def SetROIIndex(self, index):
        width, height, top, left = self.validROIS[index]

        self.AOIWidth.setValue(width)
        self.AOILeft.setValue(left)
        self.AOIHeight.setValue(height)
        self.AOITop.setValue(top)

    def SetROI(self, x1, y1, x2, y2):
        # shouldn't do GUI stuff here, but quick way of making it work
        print('Setting ROI')
        pass

    def GetROIX1(self):
        return self.AOILeft.getValue()

    def GetROIX2(self):
        return self.AOILeft.getValue() + self.AOIWidth.getValue()

    def GetROIY1(self):
        return self.AOITop.getValue()

    def GetROIY2(self):
        return self.AOITop.getValue() + self.AOIHeight.getValue()

    def DisplayError(*args):
        pass

    # def Init(*args):
    #    pass

    def Shutdown(self):
        self.pollLoopActive = False
        self.shutdown()
        # pass

    def GetStatus(*args):
        pass

    def SetCOC(*args):
        pass

    def StartExposure(self):
        # make sure no acquisiton is running
        self.StopAq()
        self._temp = self.SensorTemperature.getValue()
        self._frameRate = self.FrameRate.getValue()

        logging.info('StartAq')
        self._flush()
        self.initBuffers()
        self.AcquisitionStart()

    def StopAq(self):
        if self.CameraAcquiring.getValue():
            self.AcquisitionStop()

    def StartLifePreview(*args):
        raise Exception('Not implemented yet!!')

    def StopLifePreview(*args):
        raise Exception('Not implemented yet!!')

    def GetBWPicture(*args):
        raise Exception('Not implemented yet!!')

    def CheckCoordinates(*args):
        raise Exception('Not implemented yet!!')

    # new fcns for Andor compatibility
    def GetNumImsBuffered(self):
        return self.nFull

    def GetBufferSize(self):
        return self.nBuffers

    def SetActive(self, active=True):
        '''flag the camera as active (or inactive) to dictate whether it writes it's metadata or not'''
        self.active = active

    def GenStartMetadata(self, mdh):
        if self.active:
            self.GetStatus()

            mdh.setEntry('Camera.Name', 'Andor Neo')

            mdh.setEntry('Camera.IntegrationTime', self.GetIntegTime())
            mdh.setEntry('Camera.CycleTime', self.GetIntegTime())
            mdh.setEntry('Camera.EMGain', 1)

            mdh.setEntry('Camera.ROIPosX', self.GetROIX1())
            mdh.setEntry('Camera.ROIPosY',  self.GetROIY1())
            mdh.setEntry('Camera.ROIWidth', self.GetROIX2() - self.GetROIX1())
            mdh.setEntry('Camera.ROIHeight',  self.GetROIY2() - self.GetROIY1())
            #mdh.setEntry('Camera.StartCCDTemp',  self.GetCCDTemp())

            mdh.setEntry('Camera.ReadNoise', 1)
            mdh.setEntry('Camera.NoiseFactor', 1)
            mdh.setEntry('Camera.ElectronsPerCount', 1)
            #mdh.setEntry('Camera.ADOffset', self.noiseMaker.ADOffset)

            #mdh.setEntry('Simulation.Fluorophores', self.fluors.fl)
            #mdh.setEntry('Simulation.LaserPowers', self.laserPowers)

            #realEMGain = ccdCalibrator.getCalibratedCCDGain(self.GetEMGain(), self.GetCCDTempSetPoint())
            # if not realEMGain == None:
            mdh.setEntry('Camera.TrueEMGain', 1)

    # functions to make us look more like andor camera
    def GetEMGain(self):
        return 1

    def GetCCDTempSetPoint(self):
        return self.TargetSensorTemperature.getValue()

    def SetCCDTemp(self, temp):
        self.TargetSensorTemperature.setValue(temp)
        # pass

    def SetEMGain(self, gain):
        pass

    def SetAcquisitionMode(self, aqMode):
        self.CycleMode.setIndex(aqMode)
        self.contMode = aqMode == self.MODE_CONTINUOUS

    def SetBurst(self, burstSize):
        if burstSize > 1:
            self.SetAcquisitionMode(self.MODE_SINGLE_SHOT)
            self.FrameCount.setValue(burstSize)
            self.contMode = True
            self.burstMode = True
        else:
            self.FrameCount.setValue(1)
            self.SetAcquisitionMode(self.MODE_CONTINUOUS)
            self.burstMode = False


    def SetBaselineClamp(self, mode):
        pass

    def GetFPS(self):
        # return self.FrameRate.getValue()
        return self._frameRate

    def __del__(self):
        pass
#        self.Shutdown()
        #self.compT.kill = True


def getCameraInfos():
    """
    Gives list of connected camera models, names and serial_numbers (including simulation ones)
    Returns
    -------
    models: (list) list of string
    names: (list) list of string
    serial_numbers: (list) list of string
    """
    models = []
    names = []
    serial_numbers = []
    for ind in range(sdk3cam.getNumCameras()):
        cam = AndorBase(ind)
        sdk3cam.camReg.regCamera()
        cam.init_camera()
        try:
            models.append(cam.CameraModel.getValue())
        except:
            models.append('')
        try:
            names.append(cam.CameraName.getValue())
        except:
            names.append('')
        try:
            serial_numbers.append(cam.SerialNumber.getValue())
        except:
            serial_numbers.append('')
        cam.shutdown()
        sdk3cam.camReg.unregCamera()
    return models, names, serial_numbers


class AndorCamera(AndorBase):
    def __init__(self, camNum, nbuffer=1):
        # define properties
        self.Overlap = sdk3cam.ATBool()
        self.SpuriousNoiseFilter = sdk3cam.ATBool()

        self.CameraDump = sdk3cam.ATCommand()
        self.SoftwareTrigger = sdk3cam.ATCommand()

        self.TemperatureControl = sdk3cam.ATEnum()
        self.TemperatureStatus = sdk3cam.ATEnum()
        self.SimplePreAmpGainControl = sdk3cam.ATEnum()

        self.BitDepth = sdk3cam.ATEnum()

        self.ActualExposureTime = sdk3cam.ATFloat()
        self.BurstRate = sdk3cam.ATFloat()
        self.ReadoutTime = sdk3cam.ATFloat()

        self.AOIBinning = sdk3cam.ATEnum()
        self.AOIHBin = sdk3cam.ATInt()
        self.AOIVBin = sdk3cam.ATInt()

        self.AccumulateCount = sdk3cam.ATInt()
        self.BaselineLevel = sdk3cam.ATInt()
        self.BurstCount = sdk3cam.ATInt()
        self.LUTIndex = sdk3cam.ATInt()
        self.LUTValue = sdk3cam.ATInt()

        self.ControllerID = sdk3cam.ATString()
        self.FirmwareVersion = sdk3cam.ATString()

        super().__init__(camNum, nbuffer)


class AndorSim(AndorBase):
    def __init__(self, camNum):
        # define properties
        self.SynchronousTriggering = sdk3cam.ATBool()

        self.PixelCorrection = sdk3cam.ATEnum()
        self.TriggerSelector = sdk3cam.ATEnum()
        self.TriggerSource = sdk3cam.ATEnum()

        self.PixelHeight = sdk3cam.ATFloat()
        self.PixelWidth = sdk3cam.ATFloat()

        self.AOIHBin = sdk3cam.ATInt()
        self.AOIVbin = sdk3cam.ATInt()

        super().__init__(camNum)
