# -*- coding: utf-8 -*-
#   AndoriDus - A Python wrapper for Andor's scientific cameras
#
#   Original code by
#   Copyright (C) 2009  Hamid Ohadi
#
#   Adapted for iDus, qtlab and Windows XP
#   2010 Martijn Schaafsma
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#   Adapted for PyMoDAQ
#   2020 Sébastien Weber


# Modules for Andor functionality
import ctypes
from ctypes import windll, c_int, c_char, byref, c_long, \
    pointer, c_float, c_char_p, cdll
import sys
import time
from pathlib import Path
import platform
import os

"""
NOTE:

If controlling the shamrock through i2c it is important that both
the camera and spectrograph are being controlled through the same 
calling program and that the DLLs used are contained in the same 
working folder.

The camera MUST be initialized before attempting to communicate 
with the Shamrock.

"""
try:
    # Check operating system and load library
    if platform.system() == "Linux":
        dllpath = Path("/usr/local/lib/libandor.so")
        _dll = cdll.LoadLibrary(str(dllpath))
    elif platform.system() == "Windows":
        dllpath = Path('C:\\Program Files\\Andor SDK')
        if platform.machine() == "AMD64":
            dllname = "atmcd64d.dll"
        else:
            dllname = "atmcd32d.dll"
        dllpath = dllpath.joinpath(dllname)
        _dll = windll.LoadLibrary(str(dllpath))

except Exception as e:
    raise(f'Could not import Andor CDD library: {str(e)}')
            

# class AndorIdus(Instrument):
class AndorSDK:
    """
    Andor class which is meant to provide the Python version of the same
    functions that are defined in the Andor's SDK. Extensive documentation
    on the functions used and error codes can be
    found in the Andor SDK Users Guide
    """

    def __init__(self):
        """
        """
    @classmethod
    def init_camera(cls):
        # Initialize the device
        tekst = c_char()
        print("Initializing iDus...", )
        error = _dll.Initialize(byref(tekst))
        if error != 20002:
            raise IOError(ERROR_CODE[error])


    def __del__(self):
        _dll.ShutDown()

    def close(self):
        error = _dll.ShutDown()

    # Manage camera

    @classmethod
    def GetAvailableCameras(cls):
        Ncam = c_long()
        error = _dll.GetAvailableCameras(byref(Ncam))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return Ncam.value

    @classmethod
    def GetCameraHandle(cls, index):
        handle = c_long()
        index = c_long(index)
        error = _dll.GetCameraHandle(index, byref(handle))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return handle.value

    @classmethod
    def SetCurrentCamera(cls, handle):
        handle = c_long(handle)
        error = _dll.SetCurrentCamera(handle)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    @classmethod
    def GetCurrentCamera(cls):
        handle = c_long()
        error = _dll.GetCurrentCamera(byref(handle))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return handle.value

    @classmethod
    def GetCamerasInfo(cls):
        cameralist = []
        for ind in range(cls.GetAvailableCameras()):
            handle = cls.GetCameraHandle(ind)
            cls.SetCurrentCamera(handle)
            cls.init_camera()
            cameralist.append(dict(handle=handle,
                                   serial=cls.GetCameraSerialNumber(),
                                   model=cls.GetHeadModel()))
        return cameralist

    # Get Camera properties
    @classmethod
    def GetCameraSerialNumber(cls):
        '''
        Returns the serial number of the camera

        Input:
            None

        Output:
            (int) : Serial number of the camera
        '''
        serial = c_int()
        error = _dll.GetCameraSerialNumber(byref(serial))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return serial.value

    @classmethod
    def GetHeadModel(cls):
        '''
        Returns the camera head model name

        Input:
            None

        Output:
            (str) : model name
        '''
        model = ctypes.create_string_buffer(128)
        error = _dll.GetHeadModel(byref(model))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return model.value.decode()

    def GetDetector(self):
        '''
        Returns the camera sensor size

        Input:
            None

        Output:
            (int,int) : x and y size (hor and ver)
        '''
        xpixels = c_int()
        ypixels = c_int()
        error = _dll.GetDetector(byref(xpixels), byref(ypixels))
        if error != 20002:
            raise IOError(ERROR_CODE[error])

        return (xpixels.value, ypixels.value)

    def GetMaximumBinning(self, readmode=0, horver=1):
        '''
        Returns the max binning size in horizontal (horver=0) or vertical (horver=1) given the readout mode

        Input:
            int: readmode (see SetReadMode)
            int: 0 to retrieve horizontal binning limit, 1 to retreive limit in the vertical.

        Output:
            (str,int) : error and binning value
        '''
        maxbinning = c_int()
        error = _dll.GetMaximumBinning(readmode, horver, byref(maxbinning))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return (ERROR_CODE[error], maxbinning.value)

    def GetNumberHSSpeeds(self):
        '''
        Returns the number of HS speeds

        Input:
            None

        Output:
            (int) : the number of HS speeds
        '''
        noHSSpeeds = c_int()
        error = _dll.GetNumberHSSpeeds(self._channel, self._outamp,
                                            byref(noHSSpeeds))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return noHSSpeeds.value

    def GetNumberVSSpeeds(self):
        '''
        Returns the number of VS speeds

        Input:
            None

        Output:
            (int) : the number of VS speeds
        '''
        noVSSpeeds = c_int()
        error = _dll.GetNumberVSSpeeds(byref(noVSSpeeds))
        self._noVSSpeeds = noVSSpeeds.value
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return noVSSpeeds.value

    # Cooler and temperature
    def CoolerON(self):
        '''
        Switches the cooler on

        Input:
            None

        Output:
            None
        '''
        error = _dll.CoolerON()
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def CoolerOFF(self):
        '''
        Switches the cooler off

        Input:
            None

        Output:
            None
        '''
        error = _dll.CoolerOFF()
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def SetCoolerMode(self, mode):
        '''
        Set the cooler mode

        Input:
            mode (int) : cooler modus

        Output:
            None
        '''
        error = _dll.SetCoolerMode(mode)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def IsCoolerOn(self):
        '''
        Returns cooler status

        Input:
            None

        Output:
            (int) : Cooler status
        '''
        iCoolerStatus = c_int()
        error = _dll.IsCoolerOn(byref(iCoolerStatus))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return iCoolerStatus.value

    def GetTemperatureRange(self):
        '''
        Returns the temperature range in degrees Celcius

        Input:
            None

        Output:
            (int,int) : temperature min and max in degrees Celcius
        '''
        ctemperature_min = c_int()
        ctemperature_max = c_int()

        error = _dll.GetTemperatureRange(byref(ctemperature_min), byref(ctemperature_max))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], (ctemperature_min.value, ctemperature_max.value)

    def GetTemperature(self):
        '''
        Returns the temperature in degrees Celcius

        Input:
            None

        Output:
            (int) : temperature in degrees Celcius
        '''
        ctemperature = c_int()
        error = _dll.GetTemperature(byref(ctemperature))
        return ERROR_CODE[error], ctemperature.value

    def SetTemperature(self, temperature):
        '''
        Set the working temperature of the camera

        Input:
            temparature (int) : temperature in degrees Celcius

        Output:
            None
        '''
        #        ctemperature = c_int(temperature)
        error = _dll.SetTemperature(int(temperature))
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    ###### Single Parameters Set ######
    def SetAccumulationCycleTime(self, time_):
        '''
        Set the accumulation cycle time

        Input:
            time_ (float) : the accumulation cycle time in seconds

        Output:
            None
        '''
        error = _dll.SetAccumulationCycleTime(c_float(time_))
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def SetAcquisitionMode(self, mode):
        '''
        Set the acquisition mode of the camera

        Input:
            mode (int) : acquisition mode

        Output:
            None
        '''
        error = _dll.SetAcquisitionMode(int(mode))
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def SetDriverEvent(self, hevent):
        '''
        passes a Win32 Event handle to the SDK

        Input:
            hevent (win32 event) : event handle (created with win32event.CreateEvent

        Output:
            None
        '''
        error = _dll.SetDriverEvent(hevent)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def WaitForAcquisition(self):
        '''
        wait for acquisition to return (to be called from another thread)

        Output:
            error (str): error as a string
        '''
        error = _dll.WaitForAcquisition()
        return ERROR_CODE[error]

    def CancelWait(self):
        '''
        cancel the wait for acquisition to return

        Output:
            error (str): error as a string
        '''
        error = _dll.CancelWait()
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error]

    def SetADChannel(self, index):
        '''
        Set the A-D channel for acquisition

        Input:
            index (int) : AD channel

        Output:
            None
        '''
        error = _dll.SetADChannel(index)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def SetEMAdvanced(self, gainAdvanced):
        '''
        Enable/disable access to the advanced EM gain levels

        Input:
            gainAdvanced (int) : 1 or 0 for true or false

        Output:
            None
        '''
        error = _dll.SetEMAdvanced(gainAdvanced)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def SetEMCCDGainMode(self, gainMode):
        '''
        Set the gain mode

        Input:
            gainMode (int) : mode

        Output:
            None
        '''
        error = _dll.SetEMCCDGainMode(gainMode)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def SetExposureTime(self, time_):
        '''
        Set the exposure time in seconds

        Input:
            time_ (float) : The exposure time in seconds

        Output:
            None
        '''
        error = _dll.SetExposureTime(c_float(time_))
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def GetMaximumExposure(self):
        '''
        Get the maximum settable exposure time in seconds


        Output:
            str   : error code as string
            float : Will contain the Maximum exposure value on return.
        '''
        maxexpo = c_float()
        error = _dll.GetMaximumExposure(byref(maxexpo))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], maxexpo.value

    def SetFrameTransferMode(self, frameTransfer):
        '''
        Enable/disable the frame transfer mode

        Input:
            frameTransfer (int) : 1 or 0 for true or false

        Output:
            None
        '''
        error = _dll.SetFrameTransferMode(frameTransfer)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def SetImageRotate(self, iRotate):
        '''
        Set the modus for image rotation

        Input:
            iRotate (int) : 0 for no rotation, 1 for 90 deg cw, 2 for 90 deg ccw

        Output:
            None
        '''
        error = _dll.SetImageRotate(iRotate)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def SetKineticCycleTime(self, time_):
        '''
        Set the Kinetic cycle time in seconds

        Input:
            time_ (float) : The cycle time in seconds

        Output:
            None
        '''
        error = _dll.SetKineticCycleTime(c_float(time_))
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def SetNumberAccumulations(self, number):
        '''
        Set the number of scans accumulated in memory,
        for kinetic and accumulate modes

        Input:
            number (int) : The number of accumulations

        Output:
            None
        '''
        error = _dll.SetNumberAccumulations(int(number))
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def SetNumberKinetics(self, numKin):
        '''
        Set the number of scans accumulated in memory for kinetic mode

        Input:
            number (int) : The number of accumulations

        Output:
            None
        '''
        error = _dll.SetNumberKinetics(numKin)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def SetOutputAmplifier(self, index):
        '''
        Specify which amplifier to use if EMCCD is enabled

        Input:
            index (int) : 0 for EMCCD, 1 for conventional

        Output:
            None
        '''
        error = _dll.SetOutputAmplifier(index)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def SetReadMode(self, mode):
        '''
        Set the read mode of the camera

        Input:
            mode (int) : 0 Full Vertical Binning
                         1 Multi-Track
                         2 Random-track
                         3 Single-Track
                         4 Image

        Output:
            error
        '''
        error = _dll.SetReadMode(mode)
        self._ReadMode = mode
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error]

    def SetSingleTrack(self, center, height=10):
        '''
        Set the single track mode of the camera

        Input:
            center (int) : center pixel of the track
            height (int) : height of the track

        Output:
            error
        '''
        error = _dll.SetSingleTrack(int(center), int(height))

        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error]

    def SetMultiTrack(self, N, height, offset):
        '''
        Set the multi track mode of the camera

        Input:
            N (int) : number of tracks
            height (int) : height of each track
            offset (int) : vertical offset

        Output:
            error
            bottom (int): first pixel of first row
            gap (int) : number of rows between each track (could be 0)
        '''
        bottom = c_int()
        gap = c_int()

        error = _dll.SetMultiTrack(int(N), int(height), int(offset),
                                        byref(bottom), byref(gap))

        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], bottom.value, gap.value

    def SetImage(self, binx, biny, startx, endx, starty, endy):
        '''
        Set the image mode of the camera

        Input:
            binx (int) : binning along x
            biny (int) : binning along y
            startx (int) : left area position
            endx (int) : right area position
            starty (int) : bottom area position
            endy (int) : top area position
            
        Output:
            error
        '''
        error = _dll.SetImage(int(binx), int(biny), int(startx),
                                   int(endx), int(starty), int(endy))

        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error]

    def SetTriggerMode(self, mode):
        '''
        Set the trigger mode

        Input:
            mode (int) : 0 Internal
                         1 External
                         2 External Start (only in Fast Kinetics mode)

        Output:
            None
        '''
        error = _dll.SetTriggerMode(mode)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    ###### Single Parameters Get ######

    def GetAccumulationProgress(self):
        '''
        Returns the number of completed accumulations

        Input:
            None

        Output:
            (int) : The number of accumulations
        '''
        acc = c_long()
        series = c_long()
        error = _dll.GetAcquisitionProgress(byref(acc), byref(series))
        if ERROR_CODE[error] == "DRV_SUCCESS":
            return acc.value
        else:
            return None

    def GetAcquiredDataNumpy(self, arr_ptr, array_dim):
        '''
        Returns the Acquired data

        Input:
            arr_ptr (ctypes.c_voird_p) : array pointer from numpy: data.ctypes.data_as(ctypes.c_void_p)
            array_dim (int) : total number of pixels

        Output:
            None
        '''
        error = _dll.GetAcquiredData(arr_ptr, array_dim)
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error]

    def GetAcquiredData(self, imageArray):
        '''
        Returns the Acquired data

        Input:
            None

        Output:
            (array) : an array containing the acquired data
        '''
        if self._ReadMode == 0:
            dim = self._width
        elif self._ReadMode == 4:
            dim = self._width * self._height

        # ~ print "Dim is %s" % dim
        cimageArray = c_int * dim
        cimage = cimageArray()
        error = _dll.GetAcquiredData(pointer(cimage), dim)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

        for i in range(len(cimage)):
            imageArray.append(cimage[i])
        imageArray = imageArray[:]
        return imageArray

    def GetBitDepth(self):
        '''
        Returns the bit depth of the available channels

        Input:
            None

        Output:
            (int[]) : The bit depths
        '''
        bitDepth = c_int()
        self._bitDepths = []

        for i in range(self._noADChannels):
            _dll.GetBitDepth(i, byref(bitDepth))
            self._bitDepths.append(bitDepth.value)
        return self._bitDepths

    def GetPixelSize(self):
        '''
        Returns the camera pixel size
        Input:
            None

        Output:
            error (str)
            size (float, float) pixel size (width, height) in microns
            unsigned int WINAPI GetPixelSize(float* xSize, float* ySize)
        '''
        xsize = c_float()
        ysize = c_float()

        error = _dll.GetPixelSize(byref(xsize), byref(ysize))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return (ERROR_CODE[error], (xsize.value, ysize.value))

    def GetEMGainRange(self):
        '''
        Returns the number of completed accumulations

        Input:
            None

        Output:
            int) : The number of accumulations
        '''
        low = c_int()
        high = c_int()
        error = _dll.GetEMGainRange(byref(low), byref(high))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return low.value, high.value

    def GetNumberADChannels(self):
        '''
        Returns the number of AD channels

        Input:
            None

        Output:
            (int) : The number of AD channels
        '''
        noADChannels = c_int()
        error = _dll.GetNumberADChannels(byref(noADChannels))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return noADChannels.value

    def GetNumberPreAmpGains(self):
        '''
        Returns the number of Pre Amp Gains

        Input:
            None

        Output:
            (int) : The number of Pre Amp Gains
        '''
        noGains = c_int()
        error = _dll.GetNumberPreAmpGains(byref(noGains))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return noGains.value

    def GetSeriesProgress(self):
        '''
        Returns the number of completed kenetic scans

        Input:
            None

        Output:
            (int) : The number of completed kinetic scans
        '''
        acc = c_long()
        series = c_long()
        error = _dll.GetAcquisitionProgress(byref(acc), byref(series))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return series.value

    def GetStatus(self):
        '''
        Returns the status of the camera

        Input:
            None

        Output:
            (string) : DRV_IDLE
                       DRV_TEMPCYCLE
                       DRV_ACQUIRING
                       DRV_TIME_NOT_MET
                       DRV_KINETIC_TIME_NOT_MET
                       DRV_ERROR_ACK
                       DRV_ACQ_BUFFER
                       DRV_SPOOLERROR
        '''
        status = c_int()
        error = _dll.GetStatus(byref(status))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[status.value]

    ###### Single Parameters Get/Set ######
    def GetEMCCDGain(self):
        '''
        Returns EMCCD Gain setting

        Input:
            None

        Output:
            (int) : EMCCD gain setting
        '''
        gain = c_int()
        error = _dll.GetEMCCDGain(byref(gain))
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return gain.value

    def SetEMCCDGain(self, gain):
        '''
        Set the EMCCD Gain setting

        Input:
            gain (int) : EMCCD setting

        Output:
            None
        '''
        error = _dll.SetEMCCDGain(gain)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def GetHSSpeed(self):
        '''
        Returns the available HS speeds of the selected channel

        Input:
            None

        Output:
            (float[]) : The speeds of the selected channel
        '''
        HSSpeed = c_float()
        HSSpeeds = []
        for i in range(self.GetNumberHSSpeeds()):
            error = _dll.GetHSSpeed(self._channel, self._outamp, i, byref(HSSpeed))
            if error != 20002:
                raise IOError(ERROR_CODE[error])
            HSSpeeds.append(HSSpeed.value)
        return HSSpeeds

    def SetHSSpeed(self, index):
        '''
        Set the HS speed to the mode corresponding to the index

        Input:
            index (int) : index corresponding to the Speed mode

        Output:
            None
        '''
        error = _dll.SetHSSpeed(index)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def GetVSSpeed(self):
        '''
        Returns the available VS speeds of the selected channel

        Input:
            None

        Output:
            (float[]) : The speeds of the selected channel
        '''
        VSSpeed = c_float()
        self._VSSpeeds = []

        for i in range(self.GetVSSpeed()):
            error = _dll.GetVSSpeed(i, byref(VSSpeed))
            if error != 20002:
                raise IOError(ERROR_CODE[error])
            self._VSSpeeds.append(VSSpeed.value)
        return self._VSSpeeds

    def SetVSSpeed(self, index):
        '''
        Set the VS speed to the mode corresponding to the index

        Input:
            index (int) : index corresponding to the Speed mode

        Output:
            None
        '''
        error = _dll.SetVSSpeed(index)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def GetPreAmpGain(self):
        '''
        Returns the available Pre Amp Gains

        Input:
            None

        Output:
            (float[]) : The pre amp gains
        '''
        gain = c_float()
        preAmpGain = []

        for i in range(self.GetNumberPreAmpGains()):
            error = _dll.GetPreAmpGain(i, byref(gain))
            if error != 20002:
                raise IOError(ERROR_CODE[error])
            preAmpGain.append(gain.value)
        return preAmpGain

    def SetPreAmpGain(self, index):
        '''
        Set the Pre Amp Gain to the mode corresponding to the index

        Input:
            index (int) : index corresponding to the Gain mode

        Output:
            None
        '''
        error = _dll.SetPreAmpGain(index)
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        self._preampgain = index

    ###### iDus interaction Functions ######
    def ShutDown(self):  # Careful with this one!!
        '''
        Shut down the Andor
        '''
        error = _dll.ShutDown()
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def AbortAcquisition(self):
        '''
        Abort the acquisition
        '''
        error = _dll.AbortAcquisition()
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def StartAcquisition(self):
        '''
        Start the acquisition
        '''
        error = _dll.StartAcquisition()
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def SetSingleImage(self):
        '''
        Shortcut to apply settings for a single scan full image
        '''
        self.SetReadMode(4)
        self.SetAcquisitionMode(1)
        print("Width: %d Height: %d" % (self._width, self._height))
        self.SetImage(1, 1, 1, self._width, 1, self._height)

    def SetSingleFVB(self):
        '''
        Shortcut to apply settings for a single scan FVB
        '''
        self.SetReadMode(0)
        self.SetAcquisitionMode(1)

    def GetAcquisitionTimings(self):
        '''
        Acquire all the relevant timings for acquisition,
        and store them in local memory
        
        returns:
            str  : error as a string
            dict : dict containing all real timings. Keys: 'exposure', 'accumulate', 'kinetic' 
        '''
        exposure = c_float()
        accumulate = c_float()
        kinetic = c_float()
        error = _dll.GetAcquisitionTimings(byref(exposure),
                                                byref(accumulate), byref(kinetic))
        self._exposure = exposure.value
        self._accumulate = accumulate.value
        self._kinetic = kinetic.value
        if error != 20002:
            raise IOError(ERROR_CODE[error])
        return (ERROR_CODE[error], dict(exposure=exposure.value, accumulate=accumulate.value, kinetic=kinetic.value))

    ###### Misc functions ######

    def SetShutter(self, typ, mode, closingtime=0, openingtime=10):
        '''
        Set the configuration for the shutter

        Input:
            typ         (int) : 0/1 Output TTL low/high signal to open shutter
            mode        (int) : 0/1/2 For Auto/Open/Close
            closingtime (int) : millisecs it takes to close
            openingtime (int) : millisecs it takes to open

        Output:
            None
        '''
        error = _dll.SetShutter(typ, mode, closingtime, openingtime)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def SetShutterEx(self, typ, mode, closingtime, openingtime, extmode):
        '''
        Set the configuration for the shutter in external mode

        Input:
            typ         (int) : 0/1 Output TTL low/high signal to open shutter
            mode        (int) : 0/1/2 For Auto/Open/Close
            closingtime (int) : millisecs it takes to close
            openingtime (int) : millisecs it takes to open
            extmode     (int) : 0/1/2 For Auto/Open/Close

        Output:
            None
        '''
        error = _dll.SetShutterEx(typ, mode, closingtime, openingtime,
                                       extmode)
        if error != 20002:
            raise IOError(ERROR_CODE[error])

    def SetSpool(self, active, method, path, framebuffersize):
        '''
        Set Spooling. Refer to manual for detailed description
        '''
        error = _dll.SetSpool(active, method, c_char_p(path), framebuffersize)
        if error != 20002:
            raise IOError(ERROR_CODE[error])


#####################################################

# List of error codes
ERROR_CODE = {
    20001: "DRV_ERROR_CODES",
    20002: "DRV_SUCCESS",
    20003: "DRV_VXNOTINSTALLED",
    20006: "DRV_ERROR_FILELOAD",
    20007: "DRV_ERROR_VXD_INIT",
    20010: "DRV_ERROR_PAGELOCK",
    20011: "DRV_ERROR_PAGE_UNLOCK",
    20013: "DRV_ERROR_ACK",
    20024: "DRV_NO_NEW_DATA",
    20026: "DRV_SPOOLERROR",
    20034: "DRV_TEMP_OFF",
    20035: "DRV_TEMP_NOT_STABILIZED",
    20036: "DRV_TEMP_STABILIZED",
    20037: "DRV_TEMP_NOT_REACHED",
    20038: "DRV_TEMP_OUT_RANGE",
    20039: "DRV_TEMP_NOT_SUPPORTED",
    20040: "DRV_TEMP_DRIFT",
    20050: "DRV_COF_NOTLOADED",
    20053: "DRV_FLEXERROR",
    20066: "DRV_P1INVALID",
    20067: "DRV_P2INVALID",
    20068: "DRV_P3INVALID",
    20069: "DRV_P4INVALID",
    20070: "DRV_INIERROR",
    20071: "DRV_COERROR",
    20072: "DRV_ACQUIRING",
    20073: "DRV_IDLE",
    20074: "DRV_TEMPCYCLE",
    20075: "DRV_NOT_INITIALIZED",
    20076: "DRV_P5INVALID",
    20077: "DRV_P6INVALID",
    20083: "P7_INVALID",
    20089: "DRV_USBERROR",
    20091: "DRV_NOT_SUPPORTED",
    20099: "DRV_BINNING_ERROR",
    20990: "DRV_NOCAMERA",
    20991: "DRV_NOT_SUPPORTED",
    20992: "DRV_NOT_AVAILABLE",
}

if __name__ == '__main__':
    A = AndorSDK()
    pass
