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


# class AndorIdus(Instrument):
class AndorSDK():
    """
    Andor class which is meant to provide the Python version of the same
    functions that are defined in the Andor's SDK. Extensive documentation
    on the functions used and error codes can be
    found in the Andor SDK Users Guide
    """

    def __init__(self, dllpath=""):
        '''
        'C:\\Program Files\\Andor SDK'
        
        Loads and initializes the hardware driver.
        Initializes local parameters

        Input:
            dllpath (string)   : The path where to find the sdk2 dlls
        '''

        self._dll = None

        if not dllpath:
            dllpath = 'C:\\Program Files\\Andor SDK'
        try:
            # Check operating system and load library
            if platform.system() == "Linux":
                dllname = "/usr/local/lib/libandor.so"
                self._dll = cdll.LoadLibrary(dllname)
            elif platform.system() == "Windows":
                if platform.machine() == "AMD64":
                    dllname = "atmcd64d"
                else:
                    dllname = "atmcd32d"
                    self._dll = windll.LoadLibrary(dllname)
            else:
                print("Cannot detect operating system, will now stop")
                raise Exception("Cannot detect operating system, will now stop")
        except Exception as e:
            raise Exception("error while initialising andor libraries. " + str(e))

        # Initialize the device

        tekst = c_char()
        print("Initializing iDus...", )
        error = self._dll.Initialize(byref(tekst))
        print("%s" % (ERROR_CODE[error]))

        cw = c_int()
        ch = c_int()
        self._dll.GetDetector(byref(cw), byref(ch))

        # Initiate parameters - iDus
        self._width = cw.value
        self._height = ch.value
        self._temperature = None
        self._set_T = None
        self._gain = None
        self._gainRange = None
        self._status = ERROR_CODE[error]
        self._verbosity = False
        self._preampgain = None
        self._channel = None
        self._outamp = None
        self._hsspeed = None
        self._vsspeed = None
        self._serial = None
        self._model = None
        self._exposure = None
        self._accumulate = None
        self._kinetic = None
        self._bitDepths = []
        self._preAmpGain = []
        self._VSSpeeds = []
        self._noGains = None
        self._imageArray = []
        self._noVSSpeeds = None
        self._HSSpeeds = []
        self._noADChannels = None
        self._noHSSpeeds = None
        self._ReadMode = None

    def __del__(self):
        error = self._dll.ShutDown()
        self._verbose(ERROR_CODE[error])

    def close(self):
        error = self._dll.ShutDown()
        self._verbose(ERROR_CODE[error])

    def _verbose(self, error):
        '''
        Reports all error codes to stdout if self._verbosity=True

        Input:
            error (string)  : The string resulted from the error code
            name (string)   : The name of the function calling the device

        Output:
            None
        '''
        if self._verbosity is True:
            print("[%s]: %s" % (self.FUNC(1), error))

    def SetVerbose(self, state=True):
        '''
        Enable / disable printing error codes to stdout

        Input:
            state (bool)  : toggle verbosity, default=True

        Output:
            None
        '''
        self._verbosity = state

    # Get Camera properties

    def GetCameraSerialNumber(self):
        '''
        Returns the serial number of the camera

        Input:
            None

        Output:
            (int) : Serial number of the camera
        '''
        serial = c_int()
        error = self._dll.GetCameraSerialNumber(byref(serial))
        self._serial = serial.value
        self._verbose(ERROR_CODE[error])
        return self._serial

    def GetHeadModel(self):
        '''
        Returns the camera head model name

        Input:
            None

        Output:
            (str) : model name
        '''
        model = ctypes.create_string_buffer(128)
        error = self._dll.GetHeadModel(byref(model))
        self._model = model.value
        self._verbose(ERROR_CODE[error])
        return self._model

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
        error = self._dll.GetDetector(byref(xpixels), byref(ypixels))
        self._verbose(ERROR_CODE[error])

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
        error = self._dll.GetMaximumBinning(readmode, horver, byref(maxbinning))
        self._verbose(ERROR_CODE[error])
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
        error = self._dll.GetNumberHSSpeeds(self._channel, self._outamp,
                                            byref(noHSSpeeds))
        self._noHSSpeeds = noHSSpeeds.value
        self._verbose(ERROR_CODE[error])
        return self._noHSSpeeds

    def GetNumberVSSpeeds(self):
        '''
        Returns the number of VS speeds

        Input:
            None

        Output:
            (int) : the number of VS speeds
        '''
        noVSSpeeds = c_int()
        error = self._dll.GetNumberVSSpeeds(byref(noVSSpeeds))
        self._noVSSpeeds = noVSSpeeds.value
        self._verbose(ERROR_CODE[error])
        return self._noVSSpeeds

    # Cooler and temperature
    def CoolerON(self):
        '''
        Switches the cooler on

        Input:
            None

        Output:
            None
        '''
        error = self._dll.CoolerON()
        self._verbose(ERROR_CODE[error])

    def CoolerOFF(self):
        '''
        Switches the cooler off

        Input:
            None

        Output:
            None
        '''
        error = self._dll.CoolerOFF()
        self._verbose(ERROR_CODE[error])

    def SetCoolerMode(self, mode):
        '''
        Set the cooler mode

        Input:
            mode (int) : cooler modus

        Output:
            None
        '''
        error = self._dll.SetCoolerMode(mode)
        self._verbose(ERROR_CODE[error])

    def IsCoolerOn(self):
        '''
        Returns cooler status

        Input:
            None

        Output:
            (int) : Cooler status
        '''
        iCoolerStatus = c_int()
        error = self._dll.IsCoolerOn(byref(iCoolerStatus))
        self._verbose(ERROR_CODE[error])
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

        error = self._dll.GetTemperatureRange(byref(ctemperature_min), byref(ctemperature_max))
        self._verbose(ERROR_CODE[error])
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
        error = self._dll.GetTemperature(byref(ctemperature))
        self._temperature = ctemperature.value
        self._verbose(ERROR_CODE[error])
        # ~ print "Temperature is: %g [Set T: %g]" \
        # ~ % (self._temperature, self._set_T)
        return ERROR_CODE[error], ctemperature.value

    def SetTemperature(self, temperature):  # Fixme:, see if this works
        '''
        Set the working temperature of the camera

        Input:
            temparature (int) : temperature in degrees Celcius

        Output:
            None
        '''
        #        ctemperature = c_int(temperature)
        error = self._dll.SetTemperature(int(temperature))
        self._set_T = temperature
        self._verbose(ERROR_CODE[error])

    ###### Single Parameters Set ######
    def SetAccumulationCycleTime(self, time_):
        '''
        Set the accumulation cycle time

        Input:
            time_ (float) : the accumulation cycle time in seconds

        Output:
            None
        '''
        error = self._dll.SetAccumulationCycleTime(c_float(time_))
        self._verbose(ERROR_CODE[error])

    def SetAcquisitionMode(self, mode):
        '''
        Set the acquisition mode of the camera

        Input:
            mode (int) : acquisition mode

        Output:
            None
        '''
        error = self._dll.SetAcquisitionMode(int(mode))
        self._verbose(ERROR_CODE[error])

    def SetDriverEvent(self, hevent):
        '''
        passes a Win32 Event handle to the SDK

        Input:
            hevent (win32 event) : event handle (created with win32event.CreateEvent

        Output:
            None
        '''
        error = self._dll.SetDriverEvent(hevent)
        self._verbose(ERROR_CODE[error])

    def WaitForAcquisition(self):
        '''
        wait for acquisition to return (to be called from another thread)

        Output:
            error (str): error as a string
        '''
        error = self._dll.WaitForAcquisition()
        self._verbose(ERROR_CODE[error])
        return ERROR_CODE[error]

    def CancelWait(self):
        '''
        cancel the wait for acquisition to return

        Output:
            error (str): error as a string
        '''
        error = self._dll.CancelWait()
        self._verbose(ERROR_CODE[error])
        return ERROR_CODE[error]

    def SetADChannel(self, index):
        '''
        Set the A-D channel for acquisition

        Input:
            index (int) : AD channel

        Output:
            None
        '''
        error = self._dll.SetADChannel(index)
        self._verbose(ERROR_CODE[error])
        self._channel = index

    def SetEMAdvanced(self, gainAdvanced):
        '''
        Enable/disable access to the advanced EM gain levels

        Input:
            gainAdvanced (int) : 1 or 0 for true or false

        Output:
            None
        '''
        error = self._dll.SetEMAdvanced(gainAdvanced)
        self._verbose(ERROR_CODE[error])

    def SetEMCCDGainMode(self, gainMode):
        '''
        Set the gain mode

        Input:
            gainMode (int) : mode

        Output:
            None
        '''
        error = self._dll.SetEMCCDGainMode(gainMode)
        self._verbose(ERROR_CODE[error])

    def SetExposureTime(self, time_):
        '''
        Set the exposure time in seconds

        Input:
            time_ (float) : The exposure time in seconds

        Output:
            None
        '''
        error = self._dll.SetExposureTime(c_float(time_))
        self._verbose(ERROR_CODE[error])

    def GetMaximumExposure(self):
        '''
        Get the maximum settable exposure time in seconds


        Output:
            str   : error code as string
            float : Will contain the Maximum exposure value on return.
        '''
        maxexpo = c_float()
        error = self._dll.GetMaximumExposure(byref(maxexpo))
        self._verbose(ERROR_CODE[error])
        return ERROR_CODE[error], maxexpo.value

    def SetFrameTransferMode(self, frameTransfer):
        '''
        Enable/disable the frame transfer mode

        Input:
            frameTransfer (int) : 1 or 0 for true or false

        Output:
            None
        '''
        error = self._dll.SetFrameTransferMode(frameTransfer)
        self._verbose(ERROR_CODE[error])

    def SetImageRotate(self, iRotate):
        '''
        Set the modus for image rotation

        Input:
            iRotate (int) : 0 for no rotation, 1 for 90 deg cw, 2 for 90 deg ccw

        Output:
            None
        '''
        error = self._dll.SetImageRotate(iRotate)
        self._verbose(ERROR_CODE[error])

    def SetKineticCycleTime(self, time_):
        '''
        Set the Kinetic cycle time in seconds

        Input:
            time_ (float) : The cycle time in seconds

        Output:
            None
        '''
        error = self._dll.SetKineticCycleTime(c_float(time_))
        self._verbose(ERROR_CODE[error])

    def SetNumberAccumulations(self, number):
        '''
        Set the number of scans accumulated in memory,
        for kinetic and accumulate modes

        Input:
            number (int) : The number of accumulations

        Output:
            None
        '''
        error = self._dll.SetNumberAccumulations(int(number))
        self._verbose(ERROR_CODE[error])

    def SetNumberKinetics(self, numKin):
        '''
        Set the number of scans accumulated in memory for kinetic mode

        Input:
            number (int) : The number of accumulations

        Output:
            None
        '''
        error = self._dll.SetNumberKinetics(numKin)
        self._verbose(ERROR_CODE[error])

    def SetOutputAmplifier(self, index):
        '''
        Specify which amplifier to use if EMCCD is enabled

        Input:
            index (int) : 0 for EMCCD, 1 for conventional

        Output:
            None
        '''
        error = self._dll.SetOutputAmplifier(index)
        self._verbose(ERROR_CODE[error])
        self._outamp = index

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
        error = self._dll.SetReadMode(mode)
        self._ReadMode = mode
        self._verbose(ERROR_CODE[error])
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
        error = self._dll.SetSingleTrack(int(center), int(height))

        self._verbose(ERROR_CODE[error])
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

        error = self._dll.SetMultiTrack(int(N), int(height), int(offset),
                                        byref(bottom), byref(gap))

        self._verbose(ERROR_CODE[error])
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
        error = self._dll.SetImage(int(binx), int(biny), int(startx),
                                   int(endx), int(starty), int(endy))

        self._verbose(ERROR_CODE[error])
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
        error = self._dll.SetTriggerMode(mode)
        self._verbose(ERROR_CODE[error])

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
        error = self._dll.GetAcquisitionProgress(byref(acc), byref(series))
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
        error = self._dll.GetAcquiredData(arr_ptr, array_dim)
        self._verbose(ERROR_CODE[error])
        return ERROR_CODE[error]

    def GetAcquiredData(self, imageArray):
        '''
        Returns the Acquired data

        Input:
            None

        Output:
            (array) : an array containing the acquired data
        '''
        # FIXME : Check how this works for FVB !!!
        if self._ReadMode == 0:
            dim = self._width
        elif self._ReadMode == 4:
            dim = self._width * self._height

        # ~ print "Dim is %s" % dim
        cimageArray = c_int * dim
        cimage = cimageArray()
        error = self._dll.GetAcquiredData(pointer(cimage), dim)
        self._verbose(ERROR_CODE[error])

        for i in range(len(cimage)):
            imageArray.append(cimage[i])

        self._imageArray = imageArray[:]
        self._verbose(ERROR_CODE[error])
        return self._imageArray

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
            self._dll.GetBitDepth(i, byref(bitDepth))
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

        error = self._dll.GetPixelSize(byref(xsize), byref(ysize))
        self._verbose(ERROR_CODE[error])
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
        error = self._dll.GetEMGainRange(byref(low), byref(high))
        self._gainRange = (low.value, high.value)
        self._verbose(ERROR_CODE[error])
        return self._gainRange

    def GetNumberADChannels(self):
        '''
        Returns the number of AD channels

        Input:
            None

        Output:
            (int) : The number of AD channels
        '''
        noADChannels = c_int()
        error = self._dll.GetNumberADChannels(byref(noADChannels))
        self._noADChannels = noADChannels.value
        self._verbose(ERROR_CODE[error])
        return self._noADChannels

    def GetNumberPreAmpGains(self):
        '''
        Returns the number of Pre Amp Gains

        Input:
            None

        Output:
            (int) : The number of Pre Amp Gains
        '''
        noGains = c_int()
        error = self._dll.GetNumberPreAmpGains(byref(noGains))
        self._noGains = noGains.value
        self._verbose(ERROR_CODE[error])
        return self._noGains

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
        error = self._dll.GetAcquisitionProgress(byref(acc), byref(series))
        if ERROR_CODE[error] == "DRV_SUCCESS":
            return series.value
        else:
            return None

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
        error = self._dll.GetStatus(byref(status))
        self._status = ERROR_CODE[status.value]
        self._verbose(ERROR_CODE[error])
        return self._status

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
        error = self._dll.GetEMCCDGain(byref(gain))
        self._gain = gain.value
        self._verbose(ERROR_CODE[error])
        return self._gain

    def SetEMCCDGain(self, gain):
        '''
        Set the EMCCD Gain setting

        Input:
            gain (int) : EMCCD setting

        Output:
            None
        '''
        error = self._dll.SetEMCCDGain(gain)
        self._verbose(ERROR_CODE[error])

    def GetHSSpeed(self):
        '''
        Returns the available HS speeds of the selected channel

        Input:
            None

        Output:
            (float[]) : The speeds of the selected channel
        '''
        HSSpeed = c_float()
        self._HSSpeeds = []
        for i in range(self._noHSSpeeds):
            self._dll.GetHSSpeed(self._channel, self._outamp, i, byref(HSSpeed))
            self._HSSpeeds.append(HSSpeed.value)
        return self._HSSpeeds

    def SetHSSpeed(self, index):
        '''
        Set the HS speed to the mode corresponding to the index

        Input:
            index (int) : index corresponding to the Speed mode

        Output:
            None
        '''
        error = self._dll.SetHSSpeed(index)
        self._verbose(ERROR_CODE[error])
        self._hsspeed = index

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

        for i in range(self._noVSSpeeds):
            self._dll.GetVSSpeed(i, byref(VSSpeed))
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
        error = self._dll.SetVSSpeed(index)
        self._verbose(ERROR_CODE[error])
        self._vsspeed = index

    def GetPreAmpGain(self):
        '''
        Returns the available Pre Amp Gains

        Input:
            None

        Output:
            (float[]) : The pre amp gains
        '''
        gain = c_float()
        self._preAmpGain = []

        for i in range(self._noGains):
            self._dll.GetPreAmpGain(i, byref(gain))
            self._preAmpGain.append(gain.value)
        return self._preAmpGain

    def SetPreAmpGain(self, index):
        '''
        Set the Pre Amp Gain to the mode corresponding to the index

        Input:
            index (int) : index corresponding to the Gain mode

        Output:
            None
        '''
        error = self._dll.SetPreAmpGain(index)
        self._verbose(ERROR_CODE[error])
        self._preampgain = index

    ###### iDus interaction Functions ######
    def ShutDown(self):  # Careful with this one!!
        '''
        Shut down the Andor
        '''
        error = self._dll.ShutDown()
        self._verbose(ERROR_CODE[error])

    def AbortAcquisition(self):
        '''
        Abort the acquisition
        '''
        error = self._dll.AbortAcquisition()
        self._verbose(ERROR_CODE[error])

    def StartAcquisition(self):
        '''
        Start the acquisition
        '''
        error = self._dll.StartAcquisition()
        # self._dll.WaitForAcquisition()
        self._verbose(ERROR_CODE[error])

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
        error = self._dll.GetAcquisitionTimings(byref(exposure),
                                                byref(accumulate), byref(kinetic))
        self._exposure = exposure.value
        self._accumulate = accumulate.value
        self._kinetic = kinetic.value
        self._verbose(ERROR_CODE[error])
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
        error = self._dll.SetShutter(typ, mode, closingtime, openingtime)
        self._verbose(ERROR_CODE[error])

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
        error = self._dll.SetShutterEx(typ, mode, closingtime, openingtime,
                                       extmode)
        self._verbose(ERROR_CODE[error])

    def SetSpool(self, active, method, path, framebuffersize):
        '''
        Set Spooling. Refer to manual for detailed description
        '''
        error = self._dll.SetSpool(active, method, c_char_p(path),
                                   framebuffersize)
        self._verbose(ERROR_CODE[error])


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
