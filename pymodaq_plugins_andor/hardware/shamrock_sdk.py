import ctypes
from ctypes import windll, c_int, c_char, byref, c_long, \
    pointer, c_float, c_char_p, cdll

import sys
import platform

from pathlib import Path

class ShamrockSDK():
    """
    Andor class which is meant to provide the Python version of the same
    functions that are defined in the Andor's SDK.
    """
    def __init__(self, dllpath=""):
        '''
        'C:\\Program Files\\Andor SDK'
        
        Loads and initializes the hardware driver.
        Initializes local parameters

        Input:
            dllpath (string)   : The path where to find the sdk2 dlls
            control type (string) : either "camera", "shamrock" or "both". to be set to initialize all or part of the libraries
        '''

        self.dll = None
        if not dllpath:
            dllpath = Path('C:\\Program Files\\Andor SDK')
        else:
            dllpath = Path(dllpath)

        if platform.system() == "Windows":
            if platform.machine() == "AMD64":
                sham_path = dllpath.joinpath('Shamrock64')
            else:
                sham_path = dllpath.joinpath('Shamrock')
            sys.append(str(dllpath))
            sys.append(str(sham_path))

        try:
            # Check operating system and load library
            if platform.system() == "Windows":
                self.dll = windll.LoadLibrary('ShamrockCIF')
            else:
                print("Cannot use Shamrock on linux (yet??)")
                raise IOError("Cannot use Shamrock on linux (yet??)")
        except Exception as e:
            raise Exception("error while initialising andor libraries. " + str(e))

        nodevices = c_int()
        print("Initializing Shamrock...", )
        error = self.dll.ShamrockInitialize(dllpath)
        print("%s" % ( ERROR_CODE[error]),)
        error = self.dll.ShamrockGetNumberDevices(byref(nodevices))
        print('(Nr of devices:', nodevices.value,')')
        
        # Initiate parameters - Shamrock
        self.verbosity = False
        self.NrPixels = 0

    def __del__(self):
        errorS = self.dll.ShamrockClose()
        self._Verbose(ERROR_CODE[errorS])

    def close(self):
        errorS = self.dll.ShamrockClose()
        self._Verbose(ERROR_CODE[errorS])


    def verbose(self, error, function=''):
        if self.verbosity is True:
            print("[%s]: %s" % (function, error))
    
    def SetVerbose(self, state=True):
        '''
        Enable / disable printing error codes to stdout

        Input:
            state (bool)  : toggle verbosity, default=True

        Output:
            None
        '''
        self.verbosity = state


    def GetNumberDevicesSR(self):
        nodevices = c_int()
        error = self.dll.ShamrockGetNumberDevices(byref(nodevices))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error], nodevices.value

    def GetFunctionReturnDescriptionSR(self, error, MaxDescStrLen):
        error = c_int(error)
        MaxDescStrLen = c_int(MaxDescStrLen)
        description = c_char_p()
        err = self.dll.ShamrockGetFunctionReturnDescription(error, description, MaxDescStrLen)
        self.verbose(ERROR_CODE[err], sys._getframe().f_code.co_name)
        return ERROR_CODE[err], description.value

    #---- sdkeeprom functions

    def GetSerialNumberSR(self, device):
        device = c_int(device)
        serial = ctypes.create_string_buffer(128)
        error = self.dll.ShamrockGetSerialNumber(device, byref(serial))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error], serial.value

    def EepromGetOpticalParamsSR(self, device):
        device = c_int(device)
        FocalLength = c_float()
        AngularDeviation = c_float()
        FocalTilt = c_float()
        error = self.dll.ShamrockEepromGetOpticalParams(device, byref(FocalLength),byref(AngularDeviation), byref(FocalTilt))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error], FocalLength.value, AngularDeviation.value, FocalTilt.value

    #---- sdkgrating functions
    def SetGratingSR(self, device, grating):
        device = c_int(device)
        grating = c_int(grating)
        error = self.dll.ShamrockSetGrating(device, grating)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def GetGratingSR(self, device):
        device = c_int(device)
        grating = c_int()
        error = self.dll.ShamrockGetGrating(device, byref(grating))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error], grating.value

    def WavelengthResetSR(self, device):
        device = c_int(device)
        error = self.dll.ShamrockWavelengthReset(device)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]

    def GetNumberGratingsSR(self, device):
        device = c_int(device)
        noGratings = c_int()
        error = self.dll.ShamrockGetNumberGratings(device, byref(noGratings))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error], noGratings.value

    def GetGratingInfoSR(self, device, grating):
        device = c_int(device)
        grating = c_int(grating)
        Lines = c_float()
        Blaze = ctypes.create_string_buffer(128)
        Home = c_int()
        Offset = c_int()
        error = self.dll.ShamrockGetGratingInfo(device, grating, 
                    byref(Lines), byref(Blaze), byref(Home), byref(Offset))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error], Lines.value, Blaze.value.decode(), Home.value, Offset.value
    
    #---- sdkwavelength functions
    def SetWavelengthSR(self, device, wavelength):
        device = c_int(device)
        wavelength = c_float(wavelength)
        error = self.dll.ShamrockSetWavelength(device, wavelength)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]
    
    def GetWavelengthSR(self, device):
        device = c_int(device)
        wavelength = c_float()
        error = self.dll.ShamrockGetWavelength(device, byref(wavelength))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error], wavelength.value
    
    def GetWavelengthLimitsSR(self, device, grating):
        device = c_int(device)
        grating = c_int(grating)
        minLambda = c_float()
        maxLambda = c_float()
        error = self.dll.ShamrockGetWavelengthLimits(device, grating, byref(minLambda), byref(maxLambda))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error], minLambda.value, maxLambda.value
    
    def GotoZeroOrderSR(self, device):
        device = c_int(device)
        error = self.dll.ShamrockGotoZeroOrder(device)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]
    
    def AtZeroOrderSR(self, device):
        device = c_int(device)
        atZeroOrder = c_int()
        error = self.dll.ShamrockAtZeroOrder(device, byref(atZeroOrder))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error], atZeroOrder.value
    
    #---- sdkcalibration functions
    def SetNumberPixelsSR(self, device, NumberPixels):
        device = c_int(device)
        NumberPixels = c_int(NumberPixels)
        error = self.dll.ShamrockSetNumberPixels(device, NumberPixels)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        self.NrPixels = NumberPixels.value
        return ERROR_CODE[error]
    
    def SetPixelWidthSR(self, device, Width):
        device = c_int(device)
        Width = c_float(Width)
        error = self.dll.ShamrockSetPixelWidth(device, Width)
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error]
    
    def GetNumberPixelsSR(self, device):
        device = c_int(device)
        NumberPixels = c_int()
        error = self.dll.ShamrockGetNumberPixels(device, byref(NumberPixels))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        self.NrPixels = NumberPixels.value
        return ERROR_CODE[error], NumberPixels.value
    
    def GetPixelWidthSR(self, device):
        device = c_int(device)
        Width = c_float()
        error = self.dll.ShamrockGetPixelWidth(device, byref(Width))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error], Width.value
    
    def GetCalibrationSR(self, device, Npxls):
        device = c_int(device)
        CalibrationValues = (c_float*int(Npxls))()
        error = self.dll.ShamrockGetCalibration(device, byref(CalibrationValues), int(Npxls))
        self.verbose(ERROR_CODE[error], sys._getframe().f_code.co_name)
        return ERROR_CODE[error], CalibrationValues[:]


# ####################################################

# List of error codes
ERROR_CODE = {

    20201: "SHAMROCK_COMMUNICATION_ERROR",
    20202: "SHAMROCK_SUCCESS",
    20266: "SHAMROCK_P1INVALID",
    20267: "SHAMROCK_P2INVALID",
    20268: "SHAMROCK_P3INVALID",
    20269: "SHAMROCK_P4INVALID",
    20270: "SHAMROCK_P5INVALID",
    20275: "SHAMROCK_NOT_INITIALIZED",
    20292: "SHAMROCK_NOT_AVAILABLE"
}


