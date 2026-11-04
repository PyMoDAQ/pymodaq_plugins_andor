import platform
import sys
import os

from easydict import EasyDict as edict
import numpy as np

from time import perf_counter

from qtpy import QtWidgets, QtCore

from pymodaq_utils.utils import ThreadCommand, find_dict_in_list_from_key_val, zeros_aligned
from pymodaq_utils.logger import set_logger, get_module_name
from pymodaq_utils.config import GlobalConfig

from pymodaq_gui.parameter.utils import iter_children
from pymodaq_gui.parameter import Parameter

from pymodaq.control_modules.viewer_utility_classes import comon_parameters, main


from pymodaq_plugins_utils.hardware.camera_base_pylablib import (
    CameraBasePyLabLib, cam_params, CameraCallback)

from pylablib.devices.Andor import AndorSDK3Camera, AndorTimeoutError

from pymodaq_plugins_andor.hardware.sdk3_utils import get_camera_names

logger = set_logger(get_module_name(__file__))
config = GlobalConfig()

CAM_NAMES = get_camera_names()


cam_params.extend(
    [
        {'title': 'Encoding:', 'name': 'encoding', 'type': 'list', 'limits': []},
        {'title': 'Readout time (ms):', 'name': 'readout_time', 'type': 'float', 'value': 0., 'readonly': True},
        {'title': 'Trigger Settings:', 'name': 'trigger', 'type': 'group', 'children': [
            {'title': 'Mode:', 'name': 'trigger_mode', 'type': 'list', 'limits': []},
            {'title': 'Software Trigger:', 'name': 'soft_trigger', 'type': 'bool_push', 'value': False,
             'label': 'Fire'},
            {'title': 'External Trigger delay (ms):', 'name': 'ext_trigger_delay', 'type': 'float', 'value': 0.},
        ]},

        {'title': 'Shutter Settings:', 'name': 'shutter', 'type': 'group', 'children': [
            {'title': 'Mode:', 'name': 'shutter_mode', 'type': 'list', 'limits': []},
            {'title': 'External Trigger is:', 'name': 'shutter_on_ext_trigger', 'type': 'list', 'limits': []},
        ]},

        {'title': 'Temperature Settings:', 'name': 'temperature_settings', 'type': 'group', 'children': [
            {'title': 'Enable Cooling:', 'name': 'enable_cooling', 'type': 'bool', 'value': True},
            {'title': 'Set Point:', 'name': 'set_point', 'type': 'float', 'value': 0.},
            {'title': 'Current value:', 'name': 'current_value', 'type': 'float', 'value': 20, 'readonly': True},
            {'title': 'Status:', 'name': 'status', 'type': 'list', 'limits': [], 'readonly': True},
        ]},
    ]
)


class DAQ_2DViewer_AndorSCMOSPll(CameraBasePyLabLib):
    """
        Base class for Andor SCMOS camera


        =============== ==================
        **Attributes**   **Type**

        =============== ==================

        See Also
        --------
        utility_classes.DAQ_Viewer_base
    """

    hardware_averaging = True  # will use the accumulate acquisition mode if averaging is neccessary
    live_mode_available = True

    serial_params = [{'title': 'Camera:', 'name': 'serial_number', 'type': 'list', 'value': CAM_NAMES[0] if CAM_NAMES else '',
                      'limits': CAM_NAMES}]
    params = comon_parameters + serial_params + cam_params

    def ini_attributes(self):
        super().ini_attributes()
        self.controller: AndorSDK3Camera = None

        self.temperature_timer = QtCore.QTimer()
        self.temperature_timer.timeout.connect(self.update_temperature)

    def commit_settings(self, param: Parameter):
        """
        """
        super().commit_settings(param)

        if param.name() == 'set_point':
            self.stop()
            self.controller.set_temperature(param.value(),
                                            self.settings['temperature_settings', 'enable_cooling'])

        elif param.name() == 'encoding':
            self.controller.set_attribute_value('SimplePreAmpGainControl', param.value())

        elif param.name() in iter_children(self.settings.child('shutter'), []):
            self.set_shutter()

        elif param.name() in iter_children(self.settings.child('temperature_settings'), []):
            self.setup_temperature()

        elif param.name() == 'soft_trigger':
            if param.value():
                self.controller.set_trigger_mode('software')
                param.setValue(False)

        elif param.name() in iter_children(self.settings.child('trigger'), []):
            self.set_trigger()


    def ini_detector_custom(self, controller=None):
        """
            Initialisation procedure of the detector in four steps :
                * Register callback to get data from camera
                * Get image size and current binning
                * Set and Get temperature from camera
                * Init axes from image

            Returns
            -------
            string list ???
                The initialized status.

            See Also
            --------
            daq_utils.ThreadCommand, hardware1D.DAQ_1DViewer_Picoscope.update_pico_settings
        """

        ind_camera = self.settings.child('serial_number').opts['limits'].index(self.settings['serial_number'])
        if self.is_master:
            self.controller = AndorSDK3Camera(idx=ind_camera)

        self.setup_temperature()

        self.setup_trigger()

        self.setup_shutter()

    def update_fps(self):
        self.settings.child('timing_opts', 'fps').setValue(self.controller.get_attribute_value('FrameRate'))

    def setup_temperature(self):
        temp = self.controller.get_attribute('SensorTemperature')
        temp_status = self.controller.get_attribute('TemperatureStatus')
        self.settings.child('temperature_settings', 'status').setLimits(temp_status.values)
        self.settings.child('temperature_settings', 'set_point').setLimits((temp.min, temp.max))
        enable = self.settings['temperature_settings', 'enable_cooling']
        self.controller.set_temperature(self.settings['temperature_settings', 'set_point'], enable)

        if not self.temperature_timer.isActive():
            self.temperature_timer.start(2000)  # Timer event fired every 2s

            self.update_temperature()

    def update_temperature(self):
        """
        update temperature status and value. Fired using the temperature_timer every 2s when not grabbing
        """
        temp = self.controller.get_attribute_value('SensorTemperature')
        status = self.controller.get_attribute_value('TemperatureStatus')
        self.settings.child('temperature_settings', 'current_value').setValue(temp)
        self.settings.child('temperature_settings', 'status').setValue(status)


    def setup_trigger(self):
        trigger_mode = self.controller.get_attribute('TriggerMode')
        self.settings.child('trigger',
                            'trigger_mode').setLimits(trigger_mode.values)

        ext_trigger_delay = self.controller.get_attribute('ExternalTriggerDelay')
        self.settings.child('trigger',
                        'ext_trigger_delay').setLimits((ext_trigger_delay.min,
                                                            ext_trigger_delay.max))
        self.set_trigger()

    def set_trigger(self):
        self.controller.set_attribute_value('TriggerMode',
                                            self.settings['trigger', 'trigger_mode'])
        if 'External' in self.controller.get_attribute_value('TriggerMode'):
            ext_trigger_delay = self.controller.get_attribute('ExternalTriggerDelay')
            self.settings.child('trigger',
                                'ext_trigger_delay').setLimits((ext_trigger_delay.min,
                                                                ext_trigger_delay.max))
            self.controller.set_attribute_value('ExternalTriggerDelay',
                                                self.settings['trigger', 'ext_trigger_delay'] / 1000)
            self.settings.child('camera_settings',
                                'trigger',
                                'ext_trigger_delay').setValue(
                self.controller.get_attribute_value('ExternalTriggerDelay') * 1000)

    def setup_shutter(self):
        modes = self.controller.get_attribute('ShutterMode').values
        output_modes = self.controller.get_attribute('ShutterOutputMode').values
        self.settings.child('shutter', 'shutter_mode').setOpts(limits=modes)
        self.settings.child('shutter', 'shutter_on_ext_trigger').setOpts(limits=output_modes)
        self.set_shutter()

    def set_shutter(self):
        self.controller.set_attribute_value('ShutterMode', self.settings['shutter', 'shutter_mode'])
        self.controller.set_attribute_value('ShutterOutputMode',
                                            self.settings['shutter', 'shutter_on_ext_trigger'])

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