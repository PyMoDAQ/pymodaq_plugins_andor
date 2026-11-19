import ctypes
from ctypes import windll, c_int, c_char, byref, c_long, \
    pointer, c_float, c_char_p, c_wchar_p, cdll

import sys
import platform

from pathlib import Path

try:
    # Check operating system and load library
    if platform.system() == "Linux":
        dllpath = Path("/usr/local/lib/libandor.so")
        _dll = cdll.LoadLibrary(str(dllpath))
    elif platform.system() == "Windows":
        dllpath = Path('C:\\Program Files\\Andor SDK')
        if platform.machine() == "AMD64":
            dllpath = dllpath.joinpath("Shamrock64/ShamrockCIF.dll")
        else:
            dllpath = dllpath.joinpath("Shamrock/ShamrockCIF.dll")
        _dll = windll.LoadLibrary(str(dllpath))

except Exception as e:
    raise Exception(f'Could not import Shamrock library: {str(e)}')

# Codes relative to flipper mirrors
INPUT_CODE = 1
OUTPUT_CODE = 2

FRONT_CODE = 0
SIDE_CODE = 1


class ShamrockSDK:
    """
    Andor class which is meant to provide the Python version of the same
    functions that are defined in the Andor's SDK.
    """
    def __init__(self):
        '''
        'C:\\Program Files\\Andor SDK'
        
        Loads and initializes the hardware driver.
        Initializes local parameters

        Input:
            dllpath (string)   : The path where to find the sdk2 dlls
            control type (string) : either "camera", "shamrock" or "both". to be set to initialize all or part of the libraries
        '''

        print("Initializing Shamrock...", )
        error = _dll.ShamrockInitialize()

        nodevices = c_int()
        error = _dll.ShamrockGetNumberDevices(byref(nodevices))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        print(f'(Nr of devices: {nodevices.value})')
        
        # Initiate parameters - Shamrock
        self.NrPixels = 0

    def __del__(self):
        error = _dll.ShamrockClose()
        if error != 20202:
            raise IOError(ERROR_CODE[error])

    def close(self):
        error = _dll.ShamrockClose()
        if error != 20202:
            raise IOError(ERROR_CODE[error])

    def GetNumberDevicesSR(self):
        nodevices = c_int()
        error = _dll.ShamrockGetNumberDevices(byref(nodevices))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], nodevices.value

    def GetFunctionReturnDescriptionSR(self, error, MaxDescStrLen):
        error = c_int(error)
        MaxDescStrLen = c_int(MaxDescStrLen)
        description = c_char_p()
        error = _dll.ShamrockGetFunctionReturnDescription(error, description, MaxDescStrLen)
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], description.value

    #---- sdkeeprom functions
    def GetSerialNumberSR(self, device):
        device = c_int(device)
        serial = ctypes.create_string_buffer(128)
        error = _dll.ShamrockGetSerialNumber(device, byref(serial))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], serial.value

    def EepromGetOpticalParamsSR(self, device):
        device = c_int(device)
        FocalLength = c_float()
        AngularDeviation = c_float()
        FocalTilt = c_float()
        error = _dll.ShamrockEepromGetOpticalParams(device, byref(FocalLength),
                                                    byref(AngularDeviation), byref(FocalTilt))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], FocalLength.value, AngularDeviation.value, FocalTilt.value

    #---- sdkgrating functions
    def SetGratingSR(self, device, grating):
        device = c_int(device)
        grating = c_int(grating)
        error = _dll.ShamrockSetGrating(device, grating)
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error]

    def GetGratingSR(self, device):
        device = c_int(device)
        grating = c_int()
        error = _dll.ShamrockGetGrating(device, byref(grating))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], grating.value

    def WavelengthResetSR(self, device):
        device = c_int(device)
        error = _dll.ShamrockWavelengthReset(device)
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error]

    def GetNumberGratingsSR(self, device):
        device = c_int(device)
        noGratings = c_int()
        error = _dll.ShamrockGetNumberGratings(device, byref(noGratings))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], noGratings.value

    def GetGratingInfoSR(self, device, grating):
        device = c_int(device)
        grating = c_int(grating)
        Lines = c_float()
        Blaze = ctypes.create_string_buffer(128)
        Home = c_int()
        Offset = c_int()
        error = _dll.ShamrockGetGratingInfo(device, grating, 
                    byref(Lines), byref(Blaze), byref(Home), byref(Offset))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], Lines.value, Blaze.value.decode(), Home.value, Offset.value

    def SetGratingOffset(self, device, grating, offset):
        device = c_int(device)
        grating = c_int(grating)
        offset = c_int(offset)
        error = _dll.ShamrockSetGratingOffset(device, grating, offset)
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error]

    def GetGratingOffset(self, device, grating):
        device = c_int(device)
        grating = c_int(grating)
        offset = c_int()
        error = _dll.ShamrockGetGratingOffset(device, grating, byref(offset))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], offset.value

    #---- sdkslit functions

    def AutoSlitIsPresent(self, device, index):
        device = c_int(device)
        index = c_int(index)
        slit_present = c_int()
        error = _dll.ShamrockAutoSlitIsPresent(device, index, byref(slit_present))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], slit_present.value

    def GetAutoSlitWidthSR(self, device, index):
        device = c_int(device)
        index = c_int(index)
        sw = c_float()
        error = _dll.ShamrockGetAutoSlitWidth(device, index, byref(sw))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], sw.value

    def SetAutoSlitWidthSR(self, device, index, width):
        device = c_int(device)
        index = c_int(index)
        width = c_float(width)
        error = _dll.ShamrockSetAutoSlitWidth(device, index, width)
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error]

    #---- sdk input/output ports functions
    def get_input_port(self, device):
        device = c_int(device)
        input_port = c_int()
        error = _dll.ShamrockGetFlipperMirror(device, INPUT_CODE, byref(input_port))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], input_port.value

    def set_input_port(self, device, strinput_port):
        device = c_int(device)
        if strinput_port == "INPUT_FRONT":
            input_port = FRONT_CODE
        elif strinput_port == "INPUT_SIDE":
            input_port = SIDE_CODE
        else: output_port = 99  # we put here a value that will raise the P3_INVALID error code

        input_port = c_int(input_port)

        error = _dll.ShamrockSetFlipperMirror(device, INPUT_CODE, input_port)
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error]

    def get_output_port(self, device):
        device = c_int(device)
        output_port = c_int()
        error = _dll.ShamrockGetFlipperMirror(device, OUTPUT_CODE, byref(output_port))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], output_port.value

    def set_output_port(self, device, stroutput_port):
        device = c_int(device)
        if stroutput_port == "OUTPUT_FRONT":
            output_port = FRONT_CODE
        elif stroutput_port == "OUTPUT_SIDE":
            output_port = SIDE_CODE
        else : output_port = 99 # we put here a value that will raise the P3_INVALID error code

        output_port = c_int(output_port)

        error = _dll.ShamrockSetFlipperMirror(device, OUTPUT_CODE, output_port)
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error]

    def FlipperMirrorIsPresent(self, device, port_code):
        device = c_int(device)
        present = c_int()
        error = _dll.ShamrockFlipperMirrorIsPresent(device, port_code, byref(present))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], present.value


    #---- sdkwavelength functions
    def SetWavelengthSR(self, device, wavelength):
        device = c_int(device)
        wavelength = c_float(wavelength)
        error = _dll.ShamrockSetWavelength(device, wavelength)
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error]
    
    def GetWavelengthSR(self, device):
        device = c_int(device)
        wavelength = c_float()
        error = _dll.ShamrockGetWavelength(device, byref(wavelength))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], wavelength.value
    
    def GetWavelengthLimitsSR(self, device, grating):
        device = c_int(device)
        grating = c_int(grating)
        minLambda = c_float()
        maxLambda = c_float()
        error = _dll.ShamrockGetWavelengthLimits(device, grating, byref(minLambda), byref(maxLambda))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], minLambda.value, maxLambda.value
    
    def GotoZeroOrderSR(self, device):
        device = c_int(device)
        error = _dll.ShamrockGotoZeroOrder(device)
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error]
    
    def AtZeroOrderSR(self, device):
        device = c_int(device)
        atZeroOrder = c_int()
        error = _dll.ShamrockAtZeroOrder(device, byref(atZeroOrder))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], atZeroOrder.value
    
    #---- sdkcalibration functions
    def SetNumberPixelsSR(self, device, NumberPixels):
        device = c_int(device)
        NumberPixels = c_int(NumberPixels)
        error = _dll.ShamrockSetNumberPixels(device, NumberPixels)
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        self.NrPixels = NumberPixels.value
        return ERROR_CODE[error]
    
    def SetPixelWidthSR(self, device, Width):
        device = c_int(device)
        Width = c_float(Width)
        error = _dll.ShamrockSetPixelWidth(device, Width)
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error]
    
    def GetNumberPixelsSR(self, device):
        device = c_int(device)
        NumberPixels = c_int()
        error = _dll.ShamrockGetNumberPixels(device, byref(NumberPixels))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        self.NrPixels = NumberPixels.value
        return ERROR_CODE[error], NumberPixels.value
    
    def GetPixelWidthSR(self, device):
        device = c_int(device)
        Width = c_float()
        error = _dll.ShamrockGetPixelWidth(device, byref(Width))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
        return ERROR_CODE[error], Width.value
    
    def GetCalibrationSR(self, device, Npxls):
        device = c_int(device)
        CalibrationValues = (c_float*int(Npxls))()
        error = _dll.ShamrockGetCalibration(device, byref(CalibrationValues), int(Npxls))
        if error != 20202:
            raise IOError(ERROR_CODE[error])
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


