import numpy as np
from enum import StrEnum
from qtpy import QtWidgets, QtCore

from pymodaq_utils.utils import ThreadCommand, find_dict_in_list_from_key_val
from pymodaq_gui.parameter.utils import iter_children

from pymodaq.control_modules.viewer_utility_classes import DAQ_Viewer_base, comon_parameters, main
from pymodaq.utils.data import DataFromPlugins, Axis, DataToExport

from pymodaq_plugins_utils.hardware.camera_base_pylablib import (
    CameraBasePyLabLib, cam_params, CameraCallback)

from pylablib.devices.Andor import AndorSDK2Camera
from pymodaq_plugins_andor.hardware.sdk2_utils import get_camera_names

CAM_NAMES = get_camera_names()

class Andor_Camera_ReadOut(StrEnum):
    """
        Enum class of readout modes.

        =============== =======================
        **Attributes**    **Type**
        *names*          string list of members
        =============== =======================
    """

    FullVertBinning = 'fvb'
    SingleTrack = 'single_track'
    MultiTrack = 'multi_track'
    RandomTrack = 'random_track'
    Image = 'image'


    @classmethod
    def names(cls):
        return [name for name, member in cls.__members__.items()]


cam_params.extend(
    [
        {'title': 'Readout Modes:', 'name': 'readout', 'type': 'list', 'limits': Andor_Camera_ReadOut.names(),
         'value': 'FullVertBinning'},

        {'title': 'Readout Settings:', 'name': 'readout_settings', 'type': 'group', 'children': [

            {'title': 'Single Track Settings:', 'name': 'st_settings', 'type': 'group', 'visible': False, 'children': [
                {'title': 'Center pixel:', 'name': 'st_center', 'type': 'int', 'value': 1, 'default': 1, 'min': 1},
                {'title': 'Height:', 'name': 'st_height', 'type': 'int', 'value': 1, 'default': 1, 'min': 1},
            ]},
            {'title': 'Multi Track Settings:', 'name': 'mt_settings', 'type': 'group', 'visible': False, 'children': [
                {'title': 'Ntrack:', 'name': 'mt_N', 'type': 'int', 'value': 1, 'default': 1, 'min': 1},
                {'title': 'Height:', 'name': 'mt_height', 'type': 'int', 'value': 1, 'default': 1, 'min': 1},
                {'title': 'Offset:', 'name': 'mt_offset', 'type': 'int', 'value': 0, 'default': 0, 'min': 0},
                {'title': 'Top:', 'name': 'mt_top', 'type': 'int', 'value': 0, 'default': 0, 'min': 0,
                 'readonly': True},
                {'title': 'Gap:', 'name': 'mt_gap', 'type': 'int', 'value': 1, 'default': 1, 'min': 0,
                 'readonly': True},
            ]},
        ]},
        {'title': 'Amplification mode:', 'name': 'amp_mode', 'type': 'list', 'limits': []},
        {'title': 'Shutter Settings:', 'name': 'shutter', 'type': 'group', 'children':[
            {'title': 'Open Shutter on:', 'name': 'shutter_type', 'type': 'list', 'value': 'high', 'limits': ['low', 'high']},
            {'title': 'Shutter mode:', 'name': 'shutter_mode', 'type': 'list', 'value': 'Auto', 'limits': ['auto', 'open', 'closed']},
            {'title': 'Closing time (ms):', 'name': 'shutter_closing_time', 'type': 'int', 'value': 0, 'tip': 'millisecs it takes to close'},
            {'title': 'Opening time (ms):', 'name': 'shutter_opening_time', 'type': 'int', 'value': 10, 'tip': 'millisecs it takes to open'},
        ]},
        {'title': 'Temperature Settings:', 'name': 'temperature_settings', 'type': 'group', 'children': [
            {'title': 'Enable Cooling:', 'name': 'enable_cooling', 'type': 'bool', 'value': True},
            {'title': 'Set Point:', 'name': 'set_point', 'type': 'float', 'value': -60, 'default': -60},
            {'title': 'Current value:', 'name': 'current_value', 'type': 'float', 'value': 0, 'default': 0,
                'readonly': True},
            {'title': 'Status:', 'name': 'status', 'type': 'str', 'limits': [], 'readonly': True},
        ]},
    ]
)


class DAQ_2DViewer_AndorCCDPll(CameraBasePyLabLib):
    """
        Base class for Andor CCD camera


        =============== ==================
        **Attributes**   **Type**

        =============== ==================

        See Also
        --------
        utility_classes.DAQ_Viewer_base
    """

    hardware_averaging = True  # will use the accumulate acquisition mode if averaging is necessary

    serial_params = [{'title': 'Camera:', 'name': 'serial_number', 'type': 'list', 'value': CAM_NAMES[0],
                      'limits': CAM_NAMES}]
    params = comon_parameters + serial_params + cam_params

    def ini_attributes(self):
        super().ini_attributes()
        self.controller: AndorSDK2Camera = None

        self.ccdsize_x = None
        self.ccdsize_y = None
        self.amp_modes = []

        self.temperature_timer = QtCore.QTimer()
        self.temperature_timer.timeout.connect(self.update_temperature)

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
            if param.name() == 'set_point':
                self.controller.set_temperature(param.value())

            elif param.name() == 'readout' or param.name() in iter_children(
                    self.settings.child('readout_settings')):
                self.update_read_mode()

            elif param.name() == 'amp_mode':
                self.set_amp_mode(param.value())

            elif param.name() == 'exposure':
                self.controller.set_exposure(param.value() / 1000)  # temp should be in s
                exposure = self.controller.get_exposure()
                self.settings.child('timing_opts', 'exposure_time').setValue(exposure * 1000)
                QtWidgets.QApplication.processEvents()

            elif param.name() in iter_children(self.settings.child('camera_settings', 'shutter'), []):
                self.setup_shutter()

        except Exception as e:
            self.emit_status(ThreadCommand('Update_Status', [str(e), 'log']))

    def ini_detector_custom(self, controller=None):

        ind_camera = self.settings.child('serial_number').opts['limits'].index(self.settings['serial_number'])
        if self.is_master:
            self.controller = AndorSDK2Camera(idx=ind_camera)

        self.ccdsize_x, self.ccdsize_y = self.controller.get_detector_size()

        # set amp mode
        self.controller.init_amp_mode()
        self.settings.child('amp_mode').setLimits(self.get_all_amp_modes())
        self.settings.child('amp_mode').setValue(self.get_amp_mode())

        self.update_read_mode()
        self.setup_shutter()
        self.setup_temperature()

    def get_all_amp_modes(self):
        modes = self.controller.get_all_amp_modes()
        mode_names = []
        for mode in modes:
            name = 'CH'+str(mode.channel)+' oamp'+str(mode.oamp)+' '+f'{mode.hsspeed_MHz:.2f}'+'MHz gain='+str(mode.preamp_gain)
            mode_names.append(name)
            dict_mode = {'name': name,
                         'channel': mode.channel,
                         'oamp': mode.oamp,
                         'hsspeed': mode.hsspeed,
                         'preamp': mode.preamp}
            self.amp_modes.append(dict_mode)
        return mode_names

    def get_amp_mode(self):
        mode = self.controller.get_amp_mode()
        name = 'CH'+str(mode.channel)+' oamp'+str(mode.oamp)+' '+f'{mode.hsspeed_MHz:.2f}'+'MHz gain='+str(mode.preamp_gain)
        return name

    def set_amp_mode(self, name):
        mode_idx = next(
            i for i,dict in enumerate(self.amp_modes) if dict['name'] == name
        )
        channel = mode_idx['channel']
        oamp = mode_idx['oamp']
        hsspeed = mode_idx['hsspeed']
        preamp = mode_idx['preamp']
        self.controller.set_amp_mode(channel, oamp, hsspeed, preamp)

    def update_read_mode(self):
        read_mode = Andor_Camera_ReadOut[self.settings.child('readout').value()].value
        self.controller.set_read_mode(read_mode)

        self.settings.child('readout_settings').show()

        if read_mode == 'fvb':
            self.settings.child('readout_settings').hide()
            self.settings.child('roi').hide()
            self.settings.child('hdet').setValue(self.ccdsize_x)
            self.settings.child('vdet').setValue(1)

        elif read_mode == 'single_track':
            self.settings.child('readout_settings', 'mt_settings').hide()
            self.settings.child('readout_settings', 'st_settings').show()
            self.settings.child('roi').hide()

            center = self.settings['readout_settings', 'st_settings', 'st_center']
            width = self.settings['readout_settings', 'st_settings', 'st_height']
            self.controller.setup_single_track_mode(center, width)
            self.settings.child('hdet').setValue(self.ccdsize_x)
            self.settings.child('vdet').setValue(1)

        elif read_mode == 'multi_track':
            self.settings.child('readout_settings', 'mt_settings').show()
            self.settings.child('readout_settings', 'st_settings').hide()
            self.settings.child('roi').hide()

            number = self.settings['readout_settings', 'mt_settings', 'mt_N']
            height = self.settings['readout_settings', 'mt_settings', 'mt_height']
            offset = self.settings['readout_settings', 'mt_settings', 'mt_offset']
            number, height, offset, top, gap = self.controller.setup_multi_track_mode(number, height, offset)
            self.settings.child('readout_settings', 'mt_settings', 'mt_top').setValue(top)
            self.settings.child('readout_settings', 'mt_settings', 'mt_gap').setValue(gap)
            self.settings.child('hdet').setValue(self.ccdsize_x)
            self.settings.child('vdet').setValue(number)

        elif read_mode == 'random':
            err = 'Random mode not implemented yet'
            raise Exception(err)

        elif read_mode == 'image':
            self.settings.child('readout_settings', 'mt_settings').hide()
            self.settings.child('readout_settings', 'st_settings').hide()
            self.settings.child('roi').show()

            hstart, hend, vstart, vend, hbin, vbin = self.controller.get_roi()
            self.controller.setup_image_mode(hstart, hend, vstart, vend, hbin, vbin)
            self.settings.child('hdet').setValue(int((hend - hstart + 1) / hbin))
            self.settings.child('vdet').setValue(int((vend - vstart + 1) / vbin))

        self.compute_axes()

    def compute_axes(self):
        read_mode = Andor_Camera_ReadOut[self.settings.child('readout').value()].value

        if read_mode == 'image':
            super().compute_axes()
        else:
            Nx = self.settings['hdet']
            Ny = self.settings['vdet']
            self.x_axis = Axis(data=np.linspace(0, Nx - 1, Nx, dtype=int), label='Pixels', index=1)
            self.y_axis = Axis(data=np.linspace(0, Ny - 1, Ny, dtype=int), label='Pixels', index=0)

    def setup_shutter(self):
        mode = self.settings['shutter', 'shutter_mode']
        ttl = self.settings['shutter', 'shutter_type']
        if ttl == 'low':
            ttl_mode = 0
        elif ttl == 'high':
            ttl_mode = 1
        open_time = self.settings['shutter', 'shutter_opening_time']
        close_time = self.settings['shutter', 'shutter_closing_time']
        self.controller.setup_shutter(mode, ttl_mode, open_time, close_time)

    def setup_temperature(self):
        if not self.controller.is_cooler_on():
            self.controller.set_cooler(True)

        temp = self.controller.get_temperature_range()
        status = self.controller.get_temperature_status()
        self.settings.child('temperature_settings', 'status').setValue(status)
        self.settings.child('temperature_settings', 'set_point').setLimits((temp[0], temp[1]))
        enable = self.settings['temperature_settings', 'enable_cooling']
        self.controller.set_temperature(self.settings['temperature_settings', 'set_point'], enable)

        if not self.temperature_timer.isActive():
            self.temperature_timer.start(2000)  # Timer event fired every 2s

            self.update_temperature()

    def update_temperature(self):
        """
        update temperature status and value. Fired using the temperature_timer every 2s when not grabbing
        """
        temp = self.controller.get_temperature()
        status = self.controller.get_temperature_status()
        self.settings.child('temperature_settings', 'current_value').setValue(temp)
        self.settings.child('temperature_settings', 'status').setValue(status)

    def close(self):
        """

        """
        self.temperature_timer.stop()
        QtWidgets.QApplication.processEvents()
        if self.controller is not None:
            self.stop()
            self.controller.close()

    def grab_data(self, Naverage=1, **kwargs):
        """
            Start new acquisition in two steps :
                * Initialize data: self.data for the memory to store new data and self.data_average to store the average data
                * Start acquisition with the given exposure in ms, in "1d" or "2d" mode

            =============== =========== =============================
            **Parameters**   **Type**    **Description**
            Naverage         int         Number of images to average
            =============== =========== =============================

            See Also
            --------
            daq_utils.ThreadCommand
        """

        self.temperature_timer.stop()
        super().grab_data(Naverage, **kwargs)

    def stop(self):
        """
            stop the camera's actions.
        """
        try:
            if self.controller is not None:
                if self.controller.acquisition_in_progress():
                    self.controller.stop_acquisition()
                QtWidgets.QApplication.processEvents()
                self.temperature_timer.start(2000)

        except:
            pass
        return ""


if __name__ == '__main__':
    main(__file__, init=False)

