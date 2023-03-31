import platform
import sys

from easydict import EasyDict as edict
import numpy as np
from qtpy import QtWidgets, QtCore

from pymodaq.control_modules.viewer_utility_classes import DAQ_Viewer_base
from pymodaq.utils.logger import set_logger, get_module_name
from pymodaq.control_modules.viewer_utility_classes import comon_parameters
from pymodaq.utils.data import DataFromPlugins, Axis
from pymodaq.utils.daq_utils import ThreadCommand, find_dict_in_list_from_key_val, zeros_aligned
from pymodaq.utils.parameter.utils import iter_children

arch, plat = platform.architecture()
from time import perf_counter
logger = set_logger(get_module_name(__file__))
libpath = '/'

if plat.startswith('Windows'):
    libpath = 'C:\\Program Files\\Andor SDK3\\win32'
    if libpath not in sys.path:
        sys.path.append(libpath)

try:
    from pymodaq_plugins_andor.hardware.andor_sdk3 import api, sdk3cam
except OSError as e:
    logger.exception(e)

try:
    models, names, serial_numbers = api.getCameraInfos()
    camera_list = [f'{models[ind]} {serial_numbers[ind]}' for ind in range(len(models))]
except Exception as e:
    logger.exception(f'Impossible to communicate with camera, try to set another library path than {libpath}')
    camera_list = []


class DAQ_2DViewer_AndorSCMOS(DAQ_Viewer_base):
    """
        Base class for Andor SCMOS camera


        =============== ==================
        **Attributes**   **Type**

        =============== ==================

        See Also
        --------
        utility_classes.DAQ_Viewer_base
    """
    start_waitloop = QtCore.Signal(int, int)
    stop_waitloop = QtCore.Signal()

    hardware_averaging = True  # will use the accumulate acquisition mode if averaging is neccessary
    live_mode_available = True
    params = comon_parameters + [
        {'title': 'Dll library:', 'name': 'andor_lib', 'type': 'browsepath', 'value': libpath},

        {'title': 'Camera Settings:', 'name': 'camera_settings', 'type': 'group', 'children': [
            {'title': 'Camera Models:', 'name': 'camera_model', 'type': 'list', 'limits': camera_list},
            {'title': 'Exposure (ms):', 'name': 'exposure', 'type': 'float', 'value': 0.01, 'default': 0.01, 'min': 0},
            {'title': 'Frame Rate (Hz):', 'name': 'frame_rate', 'type': 'float', 'value': 0., 'readonly': True},
            {'title': 'Number of buffer:', 'name': 'buffer_number', 'type': 'int', 'value': 10, 'min': 1},
            {'title': '', 'name': 'reset_buffers', 'type': 'bool_push', 'value': False, 'label': 'Reset Buffers'},

            {'title': 'Sensor size:', 'name': 'image_size', 'type': 'group', 'children': [
                {'title': 'Nx:', 'name': 'Nx', 'type': 'int', 'value': 0, 'default': 0, 'readonly': True},
                {'title': 'Ny:', 'name': 'Ny', 'type': 'int', 'value': 0, 'default': 0, 'readonly': True},
                ]},
            {'title': 'Encoding:', 'name': 'encoding', 'type': 'list', 'limits': []},
            {'title': 'Image Area:', 'name': 'image_settings', 'type': 'group', 'children': [
                {'title': 'Binning:', 'name': 'binning', 'type': 'list', 'limits': []},
                {'title': 'Binning along x:', 'name': 'bin_x', 'type': 'int', 'value': 1, 'default': 1, 'min': 1},
                {'title': 'Binning along y:', 'name': 'bin_y', 'type': 'int', 'value': 1, 'default': 1, 'min': 1},
                {'title': 'Left:', 'name': 'im_left', 'type': 'int', 'value': 1, 'default': 1, 'min': 1},
                {'title': 'Top:', 'name': 'im_top', 'type': 'int', 'value': 1024, 'default': 1024, 'min': 1},
                {'title': 'Width (in binned pxls):', 'name': 'im_width', 'type': 'int', 'value': 1024, 'min': 1},
                {'title': 'Height (in binned pxls):', 'name': 'im_height', 'type': 'int', 'value': 256, 'min': 1, },
                {'title': 'Set Size Max:', 'name': 'max_size', 'type': 'bool_push', 'value': False,
                 'label': 'Max'},
                {'title': 'Readout time (ms):', 'name': 'readout_time', 'type': 'float', 'value': 0., 'readonly': True},
                ]},

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
                {'title': 'Set Point:', 'name': 'set_point', 'type': 'list', 'limits': []},
                {'title': 'Current value:', 'name': 'current_value', 'type': 'float', 'value': 20, 'readonly': True},
                {'title': 'Status:', 'name': 'status', 'type': 'list', 'limits': [], 'readonly': True},
                ]},
            ]},
        ]

    def ini_attributes(self):

        self.camera_controller: api.AndorCamera = None

        self.buffers = []
        self.buffers_pointer = []
        self._Nbuffers = None
        self._reset_buffers_cmd = False
        self.refresh_time_fr = 200

        self.current_buffer = -1
        self.n_grabed_data = None
        self.n_grabed_frame_rate = None
        self.start_time = None
        self.live = False
        self.wait_time = 0

        self.x_axis = None
        self.y_axis = None
        self.camera_controller = None
        self.data = None
        self.SIZEX, self.SIZEY = (None, None)
        self.camera_done = False
        self.acquirred_image = None
        self.callback_thread = None
        self.Naverage = None
        self.data_shape = None  # 'Data2D' if sizey != 1 else 'Data1D'

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
            self.stop()
            QtWidgets.QApplication.processEvents()

            if param.name() == 'set_point':
                self.camera_controller.SetTemperature(param.value())

            elif param.name() == 'reset_buffers':
                self._reset_buffers_cmd = True

            elif param.name() == 'exposure':
                self.camera_controller.ExposureTime.setValue(self.settings.child('camera_settings', 'exposure').value() * 1e-3)
                self.settings.child('camera_settings', 'exposure').setValue(
                    self.camera_controller.ExposureTime.getValue() * 1e3)

            elif param.name() == 'encoding':
                self.camera_controller.SimplePreAmpGainControl.setString(param.value())

            elif param.name() in iter_children(self.settings.child('camera_settings', 'shutter'), []):
                self.set_shutter()

            elif param.name() in iter_children(self.settings.child('camera_settings', 'image_settings'), []):
                if param.name() == 'binning': #if binning set from the preselection, update first the binx and biny values before setting the image area
                    self.camera_controller.AOIBinning.setString(param.value())
                    self.settings.child('camera_settings', 'image_settings', 'bin_x').setValue(
                        self.camera_controller.AOIHBin.getValue())
                    self.settings.child('camera_settings', 'image_settings', 'bin_y').setValue(
                        self.camera_controller.AOIVBin.getValue())
                    QtWidgets.QApplication.processEvents()

                if param.name() == 'bin_x':
                    self.camera_controller.AOIHBin.setValue(param.value())
                    # self.settings.child('camera_settings', 'image_settings', 'im_width').setValue(
                    #     self.controller.AOIWidth.getValue())
                    QtWidgets.QApplication.processEvents()

                if param.name() == 'bin_y':
                    self.camera_controller.AOIVBin.setValue(param.value())
                    # self.settings.child('camera_settings', 'image_settings', 'im_height').setValue(
                    #     self.controller.AOIHeight.getValue())
                    QtWidgets.QApplication.processEvents()

                if param.name() == 'max_size' and param.value():
                    #param.setValue(False)
                    #self.setup_image()
                    self.settings.child('camera_settings', 'image_settings', 'im_width').setValue(
                        self.camera_controller.AOIWidth.max())
                    # self.send_param_status(self.settings.child('camera_settings', 'image_settings', 'im_width'),
                    #                        [(self.settings.child('camera_settings', 'image_settings', 'im_width'),
                    #                         'value', self.controller.AOIWidth.max())])

                    self.settings.child('camera_settings', 'image_settings', 'im_height').setValue(
                        self.camera_controller.AOIHeight.max())

                self.setup_image()
                QtWidgets.QApplication.processEvents()
                #self.set_image_area()

            elif param.name() in iter_children(self.settings.child('camera_settings', 'temperature_settings'), []):
                self.setup_temperature()

            elif param.name() == 'soft_trigger':
                if param.value():
                    self.camera_controller.SoftwareTrigger()
                    param.setValue(False)

            elif param.name() in iter_children(self.settings.child('camera_settings', 'trigger'), []):
                self.set_trigger()

        except Exception as e:
            self.emit_status(ThreadCommand('Update_Status', [str(e), 'log']))

    def set_trigger(self):
        self.camera_controller.TriggerMode.setString(self.settings.child('camera_settings',
                                                                         'trigger', 'trigger_mode').value())
        if 'External' in self.camera_controller.TriggerMode.getString():
            self.settings.child('camera_settings', 'trigger',
                                'ext_trigger_delay').setLimits((self.camera_controller.ExternalTriggerDelay.min(),
                                                                self.camera_controller.ExternalTriggerDelay.max()))

            self.camera_controller.ExternalTriggerDelay.setValue(
                self.settings.child('camera_settings', 'trigger', 'ext_trigger_delay').value() / 1000)
            self.settings.child('camera_settings', 'trigger',
                                'ext_trigger_delay').setValue(self.camera_controller.ExternalTriggerDelay.getValue() * 1000)

    def emit_data(self, buffer_pointer):
        """
            Fonction used to emit data obtained by callback.

            See Also
            --------
            daq_utils.ThreadCommand
        """
        try:
            buff_temp = buffer_pointer[0]
            self.current_buffer += 1
            self.n_grabed_data += 1
            self.n_grabed_frame_rate += 1
            #print(f'ind_grabemit:{self.n_grabed_data}')
            self.current_buffer = self.current_buffer % self._Nbuffers
            #print(f'ind_current_buffer:{self.current_buffer}')

            if self.buffers[self.current_buffer].ctypes.data != buff_temp:
                # buff_val = [self.buffers[ind].ctypes.data for ind in range(len(self.buffers))].index(buff_temp)
                # print(f'Buffer index should be {self.current_buffer} but is in fact {buff_val}')
                self.stop()
                QtWidgets.QApplication.processEvents()

                self.emit_status(ThreadCommand('Update_Status',
                                               ['Returned buffer not equal to expected buffer,'
                                                ' restarting acquisition and'
                                                ' freeing buffers', 'log']))
                logger.warning('Returned buffer not equal to expected buffer, restarting acquisition and'
                               ' freeing buffers')
                self._reset_buffers_cmd = True
                self.grab_data(self.Naverage, live=self.live, wait_time=self.wait_time)
                return

            cam_name = self.settings.child('camera_settings', 'camera_model').value()
            Nx = self.settings.child('camera_settings', 'image_settings', 'im_width').value()
            Ny = self.settings.child('camera_settings', 'image_settings', 'im_height').value()
            data = self.camera_controller.get_image_fom_buffer(Nx, Ny, self.buffers[self.current_buffer]).T

            if self.n_grabed_data % self.Naverage == 0 and self.live:
                self.data = 1 / self.Naverage * data
            else:
                self.data += 1 / self.Naverage * data

            if not self.live:
                if self.n_grabed_data > self.Naverage:
                    self.stop()
                else:
                    #self.data += 1 / self.Naverage * data
                    if self.n_grabed_data == self.Naverage:
                        self.data_grabed_signal.emit([
                            DataFromPlugins(name=cam_name, data=[self.data], dim=self.data_shape)])

                    elif self.n_grabed_data < self.Naverage:
                        self.data_grabed_signal_temp.emit([
                            DataFromPlugins(name=cam_name, data=[self.data * self.Naverage / self.n_grabed_data],
                                            dim=self.data_shape)])
            else:  # in live mode
                if perf_counter() - self.start_time > self.refresh_time_fr / 1000:  # refresh the frame rate every
                    # refresh_time_fr ms
                    self.settings.child('camera_settings',
                                        'frame_rate').setValue(self.n_grabed_frame_rate / (self.refresh_time_fr / 1000))
                    self.start_time = perf_counter()
                    self.n_grabed_frame_rate = 0

                if self.n_grabed_data % self.Naverage == 0:
                    self.data_grabed_signal.emit([
                        DataFromPlugins(name=cam_name, data=[self.data], dim=self.data_shape)])
                else:
                    if self.n_grabed_data % self.Naverage != 0:
                        n_grabed = self.n_grabed_data % self.Naverage
                    else:
                        n_grabed = self.Naverage
                    self.data_grabed_signal_temp.emit([
                        DataFromPlugins(name=cam_name,
                                        data=[self.data * self.Naverage / n_grabed],
                                        dim=self.data_shape)])

            self.camera_controller.queue_single_buffer(self.buffers[self.current_buffer])

        except Exception as e:
            logger.exception(str(e))

    def update_read_mode(self):
        self.x_axis = self.get_xaxis()
        self.y_axis = self.get_yaxis()

    def setup_image(self):
        binnings = self.camera_controller.AOIBinning.getAvailableValues()
        self.settings.child('camera_settings', 'image_settings', 'binning').setLimits(binnings)
        self.settings.child('camera_settings', 'image_settings', 'bin_x').setLimits(
            (self.camera_controller.AOIHBin.min(), self.camera_controller.AOIHBin.max()))
        self.settings.child('camera_settings', 'image_settings', 'bin_y').setLimits(
            (self.camera_controller.AOIVBin.min(), self.camera_controller.AOIVBin.max()))
        self.settings.child('camera_settings', 'image_settings', 'im_left').setLimits(
            (self.camera_controller.AOILeft.min(), self.camera_controller.AOILeft.max()))
        self.settings.child('camera_settings', 'image_settings', 'im_top').setLimits(
            (self.camera_controller.AOITop.min(), self.camera_controller.AOITop.max()))
        self.settings.child('camera_settings', 'image_settings', 'im_width').setLimits(
            (self.camera_controller.AOIWidth.min(), self.camera_controller.AOIWidth.max()))
        self.settings.child('camera_settings', 'image_settings', 'im_height').setLimits(
            (self.camera_controller.AOIHeight.min(), self.camera_controller.AOIHeight.max()))

    def get_ROI_size_x(self):
        return self.settings.child('camera_settings', 'image_settings', 'im_width').value()

    def get_pixel_size(self):
        """Get the Macro pixel size"""
        return self.camera_controller.PixelWidth.getValue() * self.camera_controller.AOIHBin.getValue()

    def set_image_area(self):

        left = self.settings.child('camera_settings', 'image_settings', 'im_left').value()
        top = self.settings.child('camera_settings', 'image_settings', 'im_top').value()
        width = self.settings.child('camera_settings', 'image_settings', 'im_width').value()
        height = self.settings.child('camera_settings', 'image_settings', 'im_height').value()

        # first set width and height as it modifies the possible top and left
        self.camera_controller.AOIWidth.setValue(width)
        self.camera_controller.AOIHeight.setValue(height)
        self.camera_controller.AOITop.setValue(top)
        self.camera_controller.AOILeft.setValue(left)

        self.settings.child('camera_settings', 'image_settings',
                            'readout_time').setValue(self.camera_controller.ReadoutTime.getValue() * 1000)

    def ini_detector(self, controller=None):
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
        model = self.settings.child('camera_settings', 'camera_model')
        self.camera_controller = self.ini_detector_init(old_controller=controller,
                                                        new_controller=api.AndorCamera(
                                                            model.opts['limits'].index(model.value())))

        self.ini_camera()

        # %%%%%%% init axes from image
        self.x_axis = self.get_xaxis()
        self.y_axis = self.get_yaxis()

        self.data_grabed_signal_temp.emit([DataFromPlugins(name='Andor SCMOS',
                                                           data=[np.zeros((len(self.y_axis, len(self.x_axis))))],
                                                           dim='Data2D', labels=['dat0'],
                                                           x_axis=self.x_axis,
                                                           y_axis=self.y_axis), ])

        self.emit_status(ThreadCommand('close_splash'))

        info = ""
        initialized = True
        return info, initialized

    def setup_temperature(self):
        enable = self.settings.child('camera_settings', 'temperature_settings', 'enable_cooling').value()
        self.camera_controller.SensorCooling.setValue(enable)
        if not self.temperature_timer.isActive():
            self.temperature_timer.start(2000)  # Timer event fired every 2s
        if enable:
            if self.camera_controller.TemperatureControl.isWritable():
                self.camera_controller.TemperatureControl.setString(self.settings.child('camera_settings', 'temperature_settings', 'set_point').value())
            self.update_temperature()
            # set timer to update temperature info from controller


    def update_temperature(self):
        """
        update temperature status and value. Fired using the temperature_timer every 2s when not grabbing
        """
        temp = self.camera_controller.SensorTemperature.getValue()
        status = self.camera_controller.TemperatureStatus.getString()
        self.settings.child('camera_settings', 'temperature_settings', 'current_value').setValue(temp)
        self.settings.child('camera_settings', 'temperature_settings', 'status').setValue(status)

        self.settings.child('camera_settings',
                            'frame_rate').setValue(self.camera_controller.FrameRate.getValue())


    def ini_camera(self):
        sdk3cam.camReg.regCamera()
        self.camera_controller.init_camera()

        if not self.camera_controller.FullAOIControl.getValue():
            self.settings.child('camera_settings', 'image_settings').opts(readonly=True)

        self.settings.child('camera_settings', 'encoding').setOpts(
            limits=self.camera_controller.SimplePreAmpGainControl.getAvailableValues())
        self.settings.child('camera_settings', 'encoding').setValue(self.camera_controller.SimplePreAmpGainControl.getString())

        # %%%%%% Get image size and current binning
        self.SIZEX, self.SIZEY = self.camera_controller.GetCCDWidth(), self.camera_controller.GetCCDHeight()
        self.settings.child('camera_settings', 'image_settings', 'im_height').setValue(self.SIZEY)
        self.settings.child('camera_settings', 'image_settings', 'im_width').setValue(self.SIZEX)
        self.settings.child('camera_settings', 'image_settings', 'im_left').setValue(1)
        self.settings.child('camera_settings', 'image_settings', 'im_top').setValue(1)
        self.settings.child('camera_settings', 'image_size', 'Nx').setOpts(max=self.SIZEX, default=self.SIZEX, value=self.SIZEX)
        self.settings.child('camera_settings', 'image_size', 'Ny').setOpts(max=self.SIZEY, default=self.SIZEY, value=self.SIZEY)
        #
        # get max exposure range

        self.settings.child('camera_settings', 'exposure').setLimits((self.camera_controller.ExposureTime.min() * 1000,
                                                                      self.camera_controller.ExposureTime.max() * 1000))

        self.setup_image()

        # %%%%%%% Set and Get temperature from camera
        # get temperature range
        statuses = self.camera_controller.TemperatureStatus.getAvailableValues()
        self.settings.child('camera_settings', 'temperature_settings', 'status').setOpts(limits=statuses)
        values = self.camera_controller.TemperatureControl.getAvailableValues()
        self.settings.child('camera_settings', 'temperature_settings', 'set_point').setOpts(limits=values,
                                                                         readonly=self.camera_controller.TemperatureControl.isReadonly())
        self.setup_temperature()

        self.setup_trigger()

        self.setup_shutter()

        self.setup_callback()

    def setup_callback(self):
        if self.callback_thread is not None:
            if self.callback_thread.isRunning():
                self.callback_thread.terminate()

        callback = AndorCallback(self.camera_controller.wait_buffer)
        self.callback_thread = QtCore.QThread()
        callback.moveToThread(self.callback_thread)
        callback.data_sig.connect(
            self.emit_data)  # when the wait for acquisition returns (with data taken), emit_data will be fired

        self.start_waitloop.connect(callback.start)
        self.stop_waitloop.connect(callback.stop)
        self.callback_thread.callback = callback
        self.callback_thread.start()

    def setup_trigger(self):
        self.settings.child('camera_settings', 'trigger',
                            'trigger_mode').setLimits(self.camera_controller.TriggerMode.getAvailableValues())
        self.settings.child('camera_settings', 'trigger',
                            'ext_trigger_delay').setLimits((self.camera_controller.ExternalTriggerDelay.min(),
                                                            self.camera_controller.ExternalTriggerDelay.max()))
        self.set_trigger()

    def setup_shutter(self):
        modes = self.camera_controller.ShutterMode.getAvailableValues()
        output_modes = self.camera_controller.ShutterOutputMode.getAvailableValues()
        self.settings.child('camera_settings', 'shutter', 'shutter_mode').setOpts(limits=modes)
        self.settings.child('camera_settings', 'shutter', 'shutter_on_ext_trigger').setOpts(limits=output_modes)
        self.set_shutter()

    def set_shutter(self):
        self.camera_controller.ShutterMode.setString(self.settings.child('camera_settings', 'shutter', 'shutter_mode').value())
        self.camera_controller.ShutterOutputMode.setString(self.settings.child('camera_settings', 'shutter', 'shutter_on_ext_trigger').value())

    def close(self):
        """

        """
        self.temperature_timer.stop()
        QtWidgets.QApplication.processEvents()
        self.camera_controller.close()

    def get_xaxis(self):
        """
            Obtain the horizontal axis of the image.

            Returns
            -------
            1D numpy array
                Contains a vector of integer corresponding to the horizontal camera pixels.
        """
        if self.camera_controller is not None:
            # if self.control_type == "camera":
            Nx = self.settings.child('camera_settings', 'image_size', 'Nx').value()
            self.x_axis = Axis(data=np.linspace(0, Nx - 1, Nx, dtype=np.int), label='Pixels')

            self.emit_x_axis()
        else:
            raise (Exception('controller not defined'))
        return self.x_axis

    def get_yaxis(self):
        """
            Obtain the vertical axis of the image.

            Returns
            -------
            1D numpy array
                Contains a vector of integer corresponding to the vertical camera pixels.
        """
        if self.camera_controller is not None:
            Ny = self.settings.child('camera_settings', 'image_size', 'Ny').value()
            self.y_axis = Axis(data=np.linspace(0, Ny - 1, Ny, dtype=np.int), label='Pixels')
            self.emit_y_axis()
        else:
            raise (Exception('Camera not defined'))
        return self.y_axis


    def free_buffers(self):
        # empty buffers if that was not the case
        while len(self.buffers) != 0:
            self.buffers.pop(0)


    def prepare_data(self):
        sizex = self.settings.child('camera_settings', 'image_settings', 'im_width').value()
        sizey = self.settings.child('camera_settings', 'image_settings', 'im_height').value()
        if sizex != self.camera_controller.AOIWidth.getValue():
            logger.error("Memory size doesn't correspond to image size")
            return False
        if sizey != self.camera_controller.AOIHeight.getValue():
            logger.error("Memory size doesn't correspond to image size")
            return False

        # %% Initialize data: self.data for the memory to store new data and self.data_average to store the average data
        image_size = sizex * sizey
        self.data = np.zeros((sizey, sizex), dtype=np.float)

        bufSize = self.camera_controller.ImageSizeBytes.getValue()
        if not self.buffers or self.buffers[0].size != bufSize or self._reset_buffers_cmd:
            if not not self.buffers:
                self.free_buffers()
                self.camera_controller.flush()
                self._reset_buffers_cmd = False
                self.settings.child('camera_settings', 'reset_buffers').setValue(False)

            self.current_buffer = -1
            self.buffers = []
            self.buffers_pointer = []
            self._Nbuffers = self.settings.child('camera_settings', 'buffer_number').value()
            for ind in range(self._Nbuffers):
                self.buffers.append(zeros_aligned(bufSize, 8, 'uint8'))
                self.buffers_pointer.append(self.buffers[-1].ctypes.data)
                self.camera_controller.queue_single_buffer(self.buffers[-1])

        data_shape = 'Data2D' if sizey != 1 else 'Data1D'
        if data_shape != self.data_shape:
            self.data_shape = data_shape
            # init the viewers
            self.data_grabed_signal_temp.emit([DataFromPlugins(
                name=self.settings.child('camera_settings', 'camera_model').value(),
                data=[np.squeeze(self.data.reshape((sizex, sizey)).astype(np.float))], dim=self.data_shape)])

        return True

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
        try:
            self.camera_done = False
            self.Naverage = Naverage
            if 'live' in kwargs:
                self.live = kwargs['live']
            if 'wait_time' in kwargs:
                self.wait_time = kwargs['wait_time']


            self.n_grabed_data = 0
            self.n_grabed_frame_rate = 0
            self.start_time = perf_counter()  # to check if one should refresh output
            self.temperature_timer.stop()

            if self.camera_controller.CameraAcquiring.getValue():
                self.camera_controller.AcquisitionStop()

            self.set_image_area()
            status = self.prepare_data()
            if not status:
                return

            self.camera_controller.CycleMode.setString('Continuous')

            self.camera_controller.AcquisitionStart()
            if self.live:
                self.start_waitloop.emit(-1, self.wait_time)  # will trigger the waitfor acquisition
            else:
                self.start_waitloop.emit(Naverage, 0)  # will trigger the waitfor acquisition
        except Exception as e:
            self.emit_status(ThreadCommand('Update_Status', [str(e), "log"]))

    def stop(self):
        """
            stop the camera's actions.
        """
        try:
            self.stop_waitloop.emit()
            if self.camera_controller.CameraAcquiring.getValue():
                self.camera_controller.AcquisitionStop()
            QtWidgets.QApplication.processEvents()
            self.temperature_timer.start(2000)

        except:
            pass
        return ""


class AndorCallback(QtCore.QObject):
    """

    """
    data_sig = QtCore.Signal(list)

    def __init__(self, wait_fn):
        super().__init__()
        self.wait_fn = wait_fn
        self.running = False

    def start(self, naverage, wait_time=0):
        self.running = True

        self.wait_for_acquisition(naverage, wait_time)

    def stop(self):
        self.running = False

    def wait_for_acquisition(self, naverage, wait_time):
        ind_grab = 0
        while True:
            if naverage == -1:  # continuous grab
                if not self.running:
                    break
            else:
                if ind_grab >= naverage or not self.running:
                    break
            pData = self.wait_fn()
            if not not pData:
                ind_grab += 1
                #print(f'ind_grab_thread:{ind_grab}')
                self.data_sig.emit([pData])
                QtCore.QThread.msleep(wait_time)

