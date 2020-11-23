import numpy as np
from PyQt5 import QtWidgets, QtCore
from easydict import EasyDict as edict
from pymodaq.daq_viewer.utility_classes import DAQ_Viewer_base

from pymodaq.daq_utils.daq_utils import ThreadCommand, DataFromPlugins, Axis, set_logger, get_module_name, zeros_aligned
from pymodaq.daq_viewer.utility_classes import comon_parameters

from pymodaq.daq_utils.parameter.utils import iter_children
import platform
import sys
arch, plat = platform.architecture()
from time import perf_counter
logger = set_logger(get_module_name(__file__))
libpath = '/'

if plat.startswith('Windows'):
    libpath = 'C:\\Program Files\\Andor SDK3\\win32'
    if libpath not in sys.path:
        sys.path.append(libpath)

from ...hardware.andor_sdk3 import api, sdk3cam

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
    start_waitloop = QtCore.pyqtSignal()
    stop_waitloop = QtCore.pyqtSignal()

    hardware_averaging = True  # will use the accumulate acquisition mode if averaging is neccessary
    params = comon_parameters + [
        {'title': 'Dll library:', 'name': 'andor_lib', 'type': 'browsepath', 'value': libpath},

        {'title': 'Camera Settings:', 'name': 'camera_settings', 'type': 'group', 'children': [
            {'title': 'Camera Models:', 'name': 'camera_model', 'type': 'list', 'values': camera_list},
            {'title': 'Exposure (ms):', 'name': 'exposure', 'type': 'float', 'value': 0.01, 'default': 0.01, 'min': 0},
            {'title': 'Sensor size:', 'name': 'image_size', 'type': 'group', 'children': [
                {'title': 'Nx:', 'name': 'Nx', 'type': 'int', 'value': 0, 'default': 0, 'readonly': True},
                {'title': 'Ny:', 'name': 'Ny', 'type': 'int', 'value': 0, 'default': 0, 'readonly': True},
                ]},
            {'title': 'Encoding:', 'name': 'encoding', 'type': 'list', 'values': []},
            {'title': 'Image Area:', 'name': 'image_settings', 'type': 'group', 'children': [
                {'title': 'Binning:', 'name': 'binning', 'type': 'list', 'values': []},
                {'title': 'Binning along x:', 'name': 'bin_x', 'type': 'int', 'value': 1, 'default': 1, 'min': 1},
                {'title': 'Binning along y:', 'name': 'bin_y', 'type': 'int', 'value': 1, 'default': 1, 'min': 1},
                {'title': 'Left:', 'name': 'im_left', 'type': 'int', 'value': 1, 'default': 1, 'min': 1},
                {'title': 'Top:', 'name': 'im_top', 'type': 'int', 'value': 1024, 'default': 1024, 'min': 1},
                {'title': 'Width (in binned pxls):', 'name': 'im_width', 'type': 'int', 'value': 1024, 'min': 1},
                {'title': 'Height (in binned pxls):', 'name': 'im_height', 'type': 'int', 'value': 256, 'min': 1, },
                {'title': 'Set Size Max:', 'name': 'max_size', 'type': 'bool_push', 'value': False,
                 'label': 'Max'},
                ]},
            ]},
        {'title': 'Shutter Settings:', 'name': 'shutter', 'type': 'group', 'children': [
            {'title': 'Mode:', 'name': 'mode', 'type': 'list', 'values': []},
            {'title': 'External Trigger is:', 'name': 'shutter_on_ext_trigger', 'type': 'list', 'values': []},
            ]},

        {'title': 'Temperature Settings:', 'name': 'temperature_settings', 'type': 'group', 'children': [
            {'title': 'Enable Cooling:', 'name': 'enable_cooling', 'type': 'bool', 'value': True},
            {'title': 'Set Point:', 'name': 'set_point', 'type': 'list', 'values': []},
            {'title': 'Current value:', 'name': 'current_value', 'type': 'float', 'value': 20, 'readonly': True},
            {'title': 'Status:', 'name': 'status', 'type': 'list', 'values': [], 'readonly': True},
            ]},
        ]

    def __init__(self, parent=None, params_state=None):

        super().__init__(parent, params_state)  # initialize base class with commom attributes and methods

        self.buffers = []
        self.buffers_pointer = []
        self.Nbuffers = 10
        self.refresh_time = 0.2

        self.current_buffer = -1
        self.grabed_data = None
        self.start_time = None

        self.x_axis = None
        self.y_axis = None
        self.controller = None
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
            if param.name() == 'set_point':
                self.controller.SetTemperature(param.value())

            # elif param.name() == 'readout' or param.name() in iter_children(
            #         self.settings.child('camera_settings', 'readout_settings')):
            #     self.update_read_mode()

            elif param.name() == 'exposure':
                self.controller.SetIntegTime(self.settings.child('camera_settings', 'exposure').value())
                self.settings.child('camera_settings', 'exposure').setValue(self.controller.GetIntegTime())
                QtWidgets.QApplication.processEvents()

            elif param.name() == 'encoding':
                self.controller.SimplePreAmpGainControl.setString(param.value())

            elif param.name() in iter_children(self.settings.child('shutter'),
                                                                     []):
                self.set_shutter()

            elif param.name() in iter_children(
                    self.settings.child('camera_settings', 'image_settings')):
                if param.name() == 'binning': #if binning set from the preselection, update first the binx and biny values before setting the image area
                    self.controller.AOIBinning.setString(param.value())
                    self.settings.child('camera_settings', 'image_settings', 'bin_x').setValue(
                        self.controller.AOIHBin.getValue())
                    self.settings.child('camera_settings', 'image_settings', 'bin_y').setValue(
                        self.controller.AOIVBin.getValue())

                    # self.settings.child('camera_settings', 'image_settings', 'im_width').setValue(
                    #     self.controller.AOIWidth.getValue())
                    # self.settings.child('camera_settings', 'image_settings', 'im_height').setValue(
                    #     self.controller.AOIHeight.getValue())
                    QtWidgets.QApplication.processEvents()

                if param.name() == 'bin_x':
                    self.controller.AOIHBin.setValue(param.value())
                    # self.settings.child('camera_settings', 'image_settings', 'im_width').setValue(
                    #     self.controller.AOIWidth.getValue())
                    QtWidgets.QApplication.processEvents()

                if param.name() == 'bin_y':
                    self.controller.AOIVBin.setValue(param.value())
                    # self.settings.child('camera_settings', 'image_settings', 'im_height').setValue(
                    #     self.controller.AOIHeight.getValue())
                    QtWidgets.QApplication.processEvents()

                if param.name() == 'max_size' and param.value():
                    #param.setValue(False)
                    #self.setup_image()
                    self.settings.child('camera_settings', 'image_settings', 'im_width').setValue(
                        self.controller.AOIWidth.max())
                    # self.send_param_status(self.settings.child('camera_settings', 'image_settings', 'im_width'),
                    #                        [(self.settings.child('camera_settings', 'image_settings', 'im_width'),
                    #                         'value', self.controller.AOIWidth.max())])

                    self.settings.child('camera_settings', 'image_settings', 'im_height').setValue(
                        self.controller.AOIHeight.max())

                self.setup_image()
                QtWidgets.QApplication.processEvents()
                #self.set_image_area()


            elif param.name() in iter_children(
                    self.settings.child('temperature_settings')):
                self.setup_temperature()

            pass

        except Exception as e:
            self.emit_status(ThreadCommand('Update_Status', [str(e), 'log']))


    def emit_data(self, buffer_pointer):
        """
            Fonction used to emit data obtained by callback.

            See Also
            --------
            daq_utils.ThreadCommand
        """
        try:
            self.current_buffer += 1
            self.grabed_data += 1
            self.current_buffer = self.current_buffer % self.Nbuffers

            if not self.buffers[self.current_buffer].ctypes.data == buffer_pointer[0]:
                 raise RuntimeError('Returned buffer not equal to expected buffer')

            print(self.grabed_data)
            Ny = self.settings.child('camera_settings', 'image_settings', 'im_width').value()
            Nx = self.settings.child('camera_settings', 'image_settings', 'im_height').value()


            data = self.controller.get_image_fom_buffer(Nx, Ny, self.buffers[self.current_buffer])
            if (perf_counter()-self.start_time) > self.refresh_time:
                cam_name = self.settings.child('camera_settings', 'camera_model').value()
                self.data_grabed_signal_temp.emit([DataFromPlugins(name=cam_name,
                                                              data=[np.squeeze(
                                                                  data.reshape((Ny, Nx)).astype(np.float))],
                                                              dim=self.data_shape)])

                self.start_time = perf_counter()

            #QtWidgets.QApplication.processEvents()
            self.controller.queue_single_buffer(self.buffers[self.current_buffer])
              # here to be sure the timeevents are executed even if in continuous grab mode
            self.start_waitloop.emit()


        # self.controller.queue_single_buffer(self.buffer)

        except Exception as e:
            self.emit_status(ThreadCommand('Update_Status', [str(e), 'log']))

    def update_read_mode(self):

        #self.settings.child('camera_settings', 'exposure').setValue(timings['exposure'] * 1000)

        self.x_axis = self.get_xaxis()
        self.y_axis = self.get_yaxis()

    def setup_image(self):
        binnings = self.controller.AOIBinning.getAvailableValues()
        self.settings.child('camera_settings', 'image_settings', 'binning').setLimits(binnings)
        self.settings.child('camera_settings', 'image_settings', 'bin_x').setLimits(
            (self.controller.AOIHBin.min(), self.controller.AOIHBin.max()))
        self.settings.child('camera_settings', 'image_settings', 'bin_y').setLimits(
            (self.controller.AOIVBin.min(), self.controller.AOIVBin.max()))
        self.settings.child('camera_settings', 'image_settings', 'im_left').setLimits(
            (self.controller.AOILeft.min(), self.controller.AOILeft.max()))
        self.settings.child('camera_settings', 'image_settings', 'im_top').setLimits(
            (self.controller.AOITop.min(), self.controller.AOITop.max()))
        self.settings.child('camera_settings', 'image_settings', 'im_width').setLimits(
            (self.controller.AOIWidth.min(), self.controller.AOIWidth.max()))
        self.settings.child('camera_settings', 'image_settings', 'im_height').setLimits(
            (self.controller.AOIHeight.min(), self.controller.AOIHeight.max()))

    def set_image_area(self):

        left = self.settings.child('camera_settings', 'image_settings', 'im_left').value()
        top = self.settings.child('camera_settings', 'image_settings', 'im_top').value()
        width = self.settings.child('camera_settings', 'image_settings', 'im_width').value()
        height = self.settings.child('camera_settings', 'image_settings', 'im_height').value()

        # first set width and height as it modifies the possible top and left
        self.controller.AOIWidth.setValue(width)
        self.controller.AOIHeight.setValue(height)
        self.controller.AOITop.setValue(top)
        self.controller.AOILeft.setValue(left)


        # err = self.controller.SetImage(binx, biny, startx, endx, starty, endy)
        # if err == 'DRV_SUCCESS':
        #     self.settings.child('camera_settings', 'image_size', 'Nx').setValue(int((endx - startx + 1) / binx))
        #     self.settings.child('camera_settings', 'image_size', 'Ny').setValue(int((endy - starty + 1) / biny))
        #
        # return err

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
        self.status.update(edict(initialized=False, info="", x_axis=None, y_axis=None, controller=None))
        try:
            self.emit_status(ThreadCommand('show_splash', ["Initialising Camera"]))
            if self.settings.child(('controller_status')).value() == "Slave":
                if controller is None:
                    raise Exception('no controller has been defined externally while this detector is a slave one')
                else:
                    self.controller = controller
            else:
                model = self.settings.child('camera_settings', 'camera_model')
                self.controller = api.AndorCamera(model.opts['limits'].index(model.value()))

            self.ini_camera()

            # %%%%%%% init axes from image
            self.x_axis = self.get_xaxis()
            self.y_axis = self.get_yaxis()
            self.status.x_axis = self.x_axis
            self.status.y_axis = self.y_axis
            self.status.initialized = True
            self.status.controller = self.controller
            self.emit_status(ThreadCommand('close_splash'))
            return self.status

        except Exception as e:
            self.status.info = str(e)
            self.status.initialized = False
            self.emit_status(ThreadCommand('close_splash'))
            return self.status

    def setup_temperature(self):
        enable = self.settings.child('temperature_settings', 'enable_cooling').value()
        self.controller.SensorCooling.setValue(enable)
        if not self.temperature_timer.isActive():
            self.temperature_timer.start(2000)  # Timer event fired every 2s
        if enable:
            if self.controller.TemperatureControl.isWritable():
                self.controller.TemperatureControl.setString(self.settings.child('temperature_settings', 'set_point').value())
            self.update_temperature()
            # set timer to update temperature info from controller


    def update_temperature(self):
        """
        update temperature status and value. Fired using the temperature_timer every 2s when not grabbing
        """
        temp = self.controller.SensorTemperature.getValue()
        status = self.controller.TemperatureStatus.getString()
        self.settings.child('temperature_settings', 'current_value').setValue(temp)
        self.settings.child('temperature_settings', 'status').setValue(status)



    def ini_camera(self):
        sdk3cam.camReg.regCamera()
        self.controller.init_camera()

        if not self.controller.FullAOIControl.getValue():
            self.settings.child('camera_settings', 'image_settings').opts(readonly=True)

        self.settings.child('camera_settings', 'encoding').setOpts(
            limits=self.controller.SimplePreAmpGainControl.getAvailableValues())
        self.settings.child('camera_settings', 'encoding').setValue(self.controller.SimplePreAmpGainControl.getString())


        # %%%%%% Get image size and current binning
        self.SIZEX, self.SIZEY = self.controller.GetCCDWidth(), self.controller.GetCCDHeight()
        self.settings.child('camera_settings', 'image_settings', 'im_height').setValue(self.SIZEY)
        self.settings.child('camera_settings', 'image_settings', 'im_width').setValue(self.SIZEX)
        self.settings.child('camera_settings', 'image_settings', 'im_left').setValue(1)
        self.settings.child('camera_settings', 'image_settings', 'im_top').setValue(1)
        self.settings.child('camera_settings', 'image_size', 'Nx').setOpts(max=self.SIZEX, default=self.SIZEX, value=self.SIZEX)
        self.settings.child('camera_settings', 'image_size', 'Ny').setOpts(max=self.SIZEY, default=self.SIZEY, value=self.SIZEY)
        #
        # get max exposure range

        self.settings.child('camera_settings', 'exposure').setLimits((self.controller.ExposureTime.min() * 1000,
                                                                      self.controller.ExposureTime.max() * 1000))

        self.setup_image()

        # %%%%%%% Set and Get temperature from camera
        # get temperature range
        statuses = self.controller.TemperatureStatus.getAvailableValues()
        self.settings.child('temperature_settings', 'status').setOpts(limits=statuses)
        values = self.controller.TemperatureControl.getAvailableValues()
        self.settings.child('temperature_settings', 'set_point').setOpts(limits=values,
                                                            readonly=self.controller.TemperatureControl.isReadonly())
        self.setup_temperature()


        self.setup_shutter()

        callback = AndorCallback(self.controller.wait_buffer)
        self.callback_thread = QtCore.QThread()
        callback.moveToThread(self.callback_thread)
        callback.data_sig.connect(
            self.emit_data)  # when the wait for acquisition returns (with data taken), emit_data will be fired

        self.start_waitloop.connect(callback.start)
        self.stop_waitloop.connect(callback.stop)
        self.callback_thread.callback = callback
        self.callback_thread.start()

    def setup_shutter(self):
        modes = self.controller.ShutterMode.getAvailableValues()
        output_modes = self.controller.ShutterOutputMode.getAvailableValues()
        self.settings.child('shutter', 'mode').setOpts(limits=modes)
        self.settings.child('shutter', 'shutter_on_ext_trigger').setOpts(limits=output_modes)
        self.set_shutter()

    def set_shutter(self):
        self.controller.ShutterMode.setString(self.settings.child('shutter', 'mode').value())
        self.controller.ShutterOutputMode.setString(self.settings.child('shutter', 'shutter_on_ext_trigger').value())


    def close(self):
        """

        """
        self.temperature_timer.stop()
        QtWidgets.QApplication.processEvents()
        self.controller.close()

    def get_xaxis(self):
        """
            Obtain the horizontal axis of the image.

            Returns
            -------
            1D numpy array
                Contains a vector of integer corresponding to the horizontal camera pixels.
        """
        if self.controller is not None:
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
        if self.controller is not None:

            Ny = self.settings.child('camera_settings', 'image_size', 'Ny').value()
            self.y_axis = Axis(data=np.linspace(0, Ny - 1, Ny, dtype=np.int), label='Pixels')
            self.emit_y_axis()
        else:
            raise (Exception('Camera not defined'))
        return self.y_axis

    def prepare_data(self):


        sizex = self.settings.child('camera_settings', 'image_settings', 'im_width').value()
        sizey = self.settings.child('camera_settings', 'image_settings', 'im_height').value()
        if sizex != self.controller.AOIWidth.getValue():
            logger.error("Memory size doesn't correspond to image size")
            return False
        if sizey != self.controller.AOIHeight.getValue():
            logger.error("Memory size doesn't correspond to image size")
            return False


        # %%%%%% Initialize data: self.data for the memory to store new data and self.data_average to store the average data
        image_size = sizex * sizey

        self.data = np.zeros((image_size,), dtype=np.long)

        bufSize = self.controller.ImageSizeBytes.getValue()
        if not not self.buffers:
            while len(self.buffers) != 0:
                self.buffers.pop(0)
            self.controller.flush()

        self.buffers = []
        self.buffers_pointer = []

        for ind in range(self.Nbuffers):
            self.buffers.append(zeros_aligned(bufSize, 8, 'uint8'))
            self.buffers_pointer.append(self.buffers[-1].ctypes.data)
            self.controller.queue_single_buffer(self.buffers[-1])

        data_shape = 'Data2D' if sizey != 1 else 'Data1D'
        if data_shape != self.data_shape:
            self.data_shape = data_shape
            # init the viewers
            self.data_grabed_signal_temp.emit([DataFromPlugins(
                name=self.settings.child('camera_settings', 'camera_model').value(),
                data=[np.squeeze(self.data.reshape((sizey, sizex)).astype(np.float))], dim=self.data_shape)])

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
            self.current_buffer = -1
            self.grabed_data = 0
            self.start_time = perf_counter()
            self.temperature_timer.stop()

            self.Naverage = Naverage  #

            self.set_image_area()
            status = self.prepare_data()

            if not status:
                return

            # if Naverage == 1:
            #     self.controller.SetAcquisitionMode(1)
            # else:
            #     self.controller.SetAcquisitionMode(2)
            #     self.controller.SetNumberAccumulations(Naverage)
            #
            if self.controller.CameraAcquiring.getValue():
                self.controller.AcquisitionStop()
            self.controller.CycleMode.setString('Continuous')
            self.controller.AcquisitionStart()
            self.start_waitloop.emit()  # will trigger the waitfor acquisition




            # self.controller.AcquisitionStop()
            # self.controller._flush()
            #
            # data = self.controller.get_image_fom_buffer(Nx, Ny, buffer)
            # self.data_grabed_signal.emit([DataFromPlugins(name='Camera',
            #                                               data=[np.squeeze(data)],
            #                                               dim=self.data_shape)])

        except Exception as e:
            self.emit_status(ThreadCommand('Update_Status', [str(e), "log"]))

    def stop(self):
        """
            stop the camera's actions.
        """
        try:
            self.stop_waitloop.emit()
            QtCore.QThread.msleep(100)
            if self.controller.CameraAcquiring.getValue():
                self.controller.AcquisitionStop()
            self.controller.flush()
            self.temperature_timer.start(2000)


        except:
            pass
        return ""


class AndorCallback(QtCore.QObject):
    """

    """
    data_sig = QtCore.pyqtSignal(list)

    def __init__(self, wait_fn):
        super().__init__()
        self.wait_fn = wait_fn
        self.running = False

    def start(self):
        self.running = True
        self.wait_for_acquisition()

    def stop(self):
        self.running = False

    def wait_for_acquisition(self):
        pData = self.wait_fn()
        if not not pData:
            self.data_sig.emit([pData])


