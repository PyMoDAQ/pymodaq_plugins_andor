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


    def init_camera(self):
        super().init_camera()

    def _flush(self):
        sdk3.Flush(self.handle)

    def queue_single_buffer(self, buf):
        sdk3.QueueBuffer(self.handle, buf.ctypes.data_as(sdk3.POINTER(sdk3.AT_U8)), buf.nbytes)


    def wait_buffer(self):
        try:
            pData, lData = sdk3.WaitBuffer(self.handle, 0)
        except sdk3.TimeoutError as e:
            return
        except sdk3.CameraError as e:
            #    if not e.errNo == sdk3.AT_ERR_NODATA:
            #        traceback.print_exc()
            return
        return ctypes.addressof(pData.contents)

    def get_image_fom_buffer(self, Ny, Nx, buffer):
        data = np.empty((Ny, Nx), np.uint16)

        a_s = self.AOIStride.getValue()
        dt = self.PixelEncoding.getString()
        sdk3.ConvertBuffer(buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
                           data.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
                           Ny, Nx, a_s, dt, 'Mono16')

        return data


    def flush(self):
        self._flush()

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

    def GetCCDTemp(self):
        # for some reason querying the temperature takes a lot of time - do it less often
        # return self.SensorTemperature.getValue()

        return self._temp

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

    def StopAq(self):
        if self.CameraAcquiring.getValue():
            self.AcquisitionStop()

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


    def GetFPS(self):
        return self.FrameRate.getValue()


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
        self.ExternalTriggerDelay = sdk3cam.ATFloat()

        self.AOIBinning = sdk3cam.ATEnum()
        self.AOIHBin = sdk3cam.ATInt()
        self.AOIVBin = sdk3cam.ATInt()

        self.PixelWidth = sdk3cam.ATFloat()
        self.PixelHeight = sdk3cam.ATFloat()

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
