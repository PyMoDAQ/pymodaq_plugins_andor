import sys
import numpy as np
import ctypes
import platform
from easydict import EasyDict as edict
from pymodaq.daq_move.utility_classes import DAQ_Move_base, comon_parameters
from pymodaq.daq_utils.daq_utils import ThreadCommand
from ..hardware.shamrock_sdk import ShamrockSDK
from pathlib import Path

is_64bits = sys.maxsize > 2 ** 32

if platform.system() == "Linux":
    libpath = ctypes.util.find_library('libandor')  # to be checked
elif platform.system() == "Windows":
    if is_64bits:
        libpath = ctypes.util.find_library('atmcd64d')
        if libpath is not None:
            libpath = Path(libpath).joinpath('Shamrock64')
    else:
        libpath = ctypes.util.find_library('atmcd32d')
        if libpath is not None:
            libpath = Path(libpath).joinpath('Shamrock')
if libpath is None:
    libpath = ""


class DAQ_Move_Shamrock(DAQ_Move_base):
    """
        Base class for Shamrock Spectrometer spectrometer


        =============== ==================
        **Attributes**   **Type**

        =============== ==================
    """
    _controller_units = 'nm'
    is_multiaxes = False  # set to True if this plugin is controlled for a multiaxis controller (with a unique communication link)
    stage_names = []  # "list of strings of the multiaxes

    params = [
                 {'title': 'Dll library:', 'name': 'andor_lib', 'type': 'browsepath', 'value': libpath},
                 {'title': 'Spectro Settings:', 'name': 'spectro_settings', 'type': 'group', 'expanded': True,
                  'children': [
                      {'title': 'Spectro SN:', 'name': 'spectro_serialnumber', 'type': 'str', 'value': '',
                       'readonly': True},
                      {'title': 'Wavelength (nm):', 'name': 'spectro_wl', 'type': 'float', 'value': 600, 'min': 0,
                       'readonly': True},
                      {'title': 'Grating Settings:', 'name': 'grating_settings', 'type': 'group', 'expanded': True,
                       'children': [
                           {'title': 'Grating:', 'name': 'grating', 'type': 'list'},
                           {'title': 'Lines (/mm):', 'name': 'lines', 'type': 'int', 'readonly': True},
                           {'title': 'Blaze WL (nm):', 'name': 'blaze', 'type': 'str', 'readonly': True},
                       ]},
                      {'title': 'Flip wavelength axis:', 'name': 'flip_wavelength', 'type': 'bool', 'value': False,
                       'visible': False},
                      {'title': 'Go to zero order:', 'name': 'zero_order', 'type': 'bool'},
                  ]},
             ] + comon_parameters

    def __init__(self, parent=None, params_state=None):

        super().__init__(parent, params_state)  # initialize base class with commom attributes and methods

    def commit_settings(self, param):
        """
            | Activate parameters changes on the hardware from parameter's name.
            |

            =============== ================================    =========================
            **Parameters**   **Type**                           **Description**
            *param*          instance of pyqtgraph parameter    The parameter to activate
            =============== ================================    =========================

            Three profile of parameter :
                * **bin_x** : set binning camera from bin_x parameter's value
                * **bin_y** : set binning camera from bin_y parameter's value
                * **set_point** : Set the camera's temperature from parameter's value.

        """
        try:
            if param.name() == 'grating':
                index_grating = self.grating_list.index(param.value())
                self.get_set_grating(index_grating)
                self.set_wavelength(self.settings.child('spectro_settings', 'spectro_wl').value())

            elif param.name() == 'spectro_wl':
                self.set_wavelength(param.value())

            elif param.name() == 'zero_order':
                if param.value():
                    param.setValue(False)
                    err = self.controller.GotoZeroOrderSR(0)
                    if err != 'SHAMROCK_SUCCESS':
                        raise Exception(err)

        except Exception as e:
            self.emit_status(ThreadCommand('Update_Status', [str(e), 'log']))

    def set_wavelength(self, wavelength):
        self.emit_status(ThreadCommand('show_splash', ["Setting wavelength, please wait!"]))
        err = self.controller.SetWavelengthSR(0, wavelength)
        self.emit_status(ThreadCommand('close_splash'))

        if err != 'SHAMROCK_SUCCESS':
            raise IOError(err)

        self.get_wavelength()

    def get_wavelength(self):
        err, wl = self.controller.GetWavelengthSR(0)
        if err == "SHAMROCK_SUCCESS":
            self.settings.child('spectro_settings', 'spectro_wl').setValue(wl)
        return wl

    def ini_stage(self, controller=None):
        """Actuator communication initialization

        Parameters
        ----------
        controller: (object) custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller (Master case)

        Returns
        -------
        self.status (edict): with initialization status: three fields:
            * info (str)
            * controller (object) initialized controller
            *initialized: (bool): False if initialization failed otherwise True
        """
        self.status.update(edict(initialized=False, info="", controller=None))
        try:
            self.emit_status(ThreadCommand('show_splash', ["Initialising Shamrock"]))
            if self.settings.child(('controller_status')).value() == "Slave":
                if controller is None:
                    raise Exception('no controller has been defined externally while this detector is a slave one')
                else:
                    self.controller = controller
            else:
                self.controller = ShamrockSDK(self.settings.child(('andor_lib')).value())

            self.emit_status(ThreadCommand('show_splash', ["Set/Get Shamrock's settings"]))
            self.ini_spectro()

            self.status.initialized = True
            self.status.controller = self.controller
            self.emit_status(ThreadCommand('close_splash'))
            return self.status

        except Exception as e:
            self.status.info = str(e)
            self.status.initialized = False
            self.emit_status(ThreadCommand('close_splash'))
            return self.status

    def ini_spectro(self):
        self.settings.child('spectro_settings', 'spectro_serialnumber').setValue(
            self.controller.GetSerialNumberSR(0)[1].decode())

        # get grating info
        (err, Ngratings) = self.controller.GetNumberGratingsSR(0)
        self.grating_list = []
        for ind_grating in range(1, Ngratings + 1):
            (err, lines, blaze, home, offset) = self.controller.GetGratingInfoSR(0, ind_grating)
            self.grating_list.append(str(int(lines)))

        self.settings.child('spectro_settings', 'grating_settings', 'grating').setLimits(self.grating_list)
        err, ind_grating = self.controller.GetGratingSR(0)
        self.settings.child('spectro_settings', 'grating_settings', 'grating').setValue(
            self.grating_list[ind_grating - 1])

        self.get_set_grating(ind_grating - 1)

    def check_position(self):
        """Get the current position from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        pos = self.get_wavelength()
        ##

        pos = self.get_position_with_scaling(pos)
        self.emit_status(ThreadCommand('check_position', [pos]))
        return pos

    def move_Abs(self, position):
        """ Move the actuator to the absolute target defined by position

        Parameters
        ----------
        position: (flaot) value of the absolute target positioning
        """

        position = self.check_bound(position)  # if user checked bounds, the defined bounds are applied here
        position = self.set_position_with_scaling(position)  # apply scaling if the user specified one

        self.set_wavelength(position)
        self.emit_status(ThreadCommand('Update_Status', ['Some info you want to log']))
        ##############################

        self.target_position = position
        self.poll_moving()  # start a loop to poll the current actuator value and compare it with target position

    def move_Rel(self, position):
        """ Move the actuator to the relative target actuator value defined by position

        Parameters
        ----------
        position: (flaot) value of the relative target positioning
        """
        position = self.check_bound(self.current_position + position) - self.current_position
        self.target_position = position + self.current_position

        self.set_wavelength(self.target_position)

        self.emit_status(ThreadCommand('Update_Status', ['Some info you want to log']))
        ##############################

        self.poll_moving()

    def move_Home(self):
        """

        """
        self.controller.set_wavelength(np.mean((self.settings.child('spectro_settings', 'spectro_wl').opts['limits'])))
        self.emit_status(ThreadCommand('Update_Status', ['Some info you want to log']))
        ##############################

    def stop_motion(self):
        """
        Call the specific move_done function (depending on the hardware).

        See Also
        --------
        move_done
        """

        self.move_done()  # to let the interface know the actuator stopped. Direct call as the setwavelength call is
        # blocking anyway

    def get_set_grating(self, ind_grating):
        """
        set the current grating to ind_grating+1. ind_grating corresponds to the index in the GUI graitng list while the SDK index starts at 1...

        """
        self.emit_status(ThreadCommand('show_splash', ["Moving grating please wait"]))
        err = self.controller.SetGratingSR(0, ind_grating + 1)
        err, ind_grating = self.controller.GetGratingSR(0)

        (err, lines, blaze, home, offset) = self.controller.GetGratingInfoSR(0, ind_grating)
        self.settings.child('spectro_settings', 'grating_settings', 'grating').setValue(
            self.grating_list[ind_grating - 1])
        self.settings.child('spectro_settings', 'grating_settings', 'lines').setValue(lines)
        self.settings.child('spectro_settings', 'grating_settings', 'blaze').setValue(blaze)

        (err, wl_min, wl_max) = self.controller.GetWavelengthLimitsSR(0, ind_grating)

        if err == "SHAMROCK_SUCCESS":
            self.settings.child('spectro_settings', 'spectro_wl').setOpts(limits=(wl_min, wl_max),
                                                                          tip=f'Possible values are within {wl_min} and {wl_max} for the selected grating')

        self.emit_status(ThreadCommand('close_splash'))

    def close(self):
        """

        """
        self.controller.close()

    def stop(self):
        """

        """
        return ""
