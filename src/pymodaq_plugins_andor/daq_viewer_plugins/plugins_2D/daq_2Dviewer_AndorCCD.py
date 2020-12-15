import numpy as np
from enum import IntEnum
import ctypes
from ctypes.util import find_library
import platform
from PyQt5 import QtWidgets, QtCore
from easydict import EasyDict as edict
from pymodaq.daq_viewer.utility_classes import DAQ_Viewer_base

from pymodaq.daq_utils.daq_utils import ThreadCommand, DataFromPlugins, Axis, find_dict_in_list_from_key_val
from pymodaq.daq_viewer.utility_classes import comon_parameters
from pymodaq.daq_utils.parameter.utils import iter_children


from ...hardware.andor_sdk2 import sdk2
libpath = sdk2.dllpath
camera_list = sdk2.AndorSDK.GetCamerasInfo()


class Andor_Camera_ReadOut(IntEnum):
    """
        Enum class of readout modes.

        =============== =======================
        **Attributes**    **Type**
        *names*          string list of members
        =============== =======================
    """
    
    FullVertBinning=0
    SingleTrack=3
    MultiTrack=1
    RandomTrack=2
    Image=4
    Cropped=5


    @classmethod
    def names(cls):
        return [name for name, member in cls.__members__.items()]

class Andor_Camera_AcqMode(IntEnum):
    """
        Enum class of AcqModes modes.

        =============== =======================
        **Attributes**    **Type**
        *names*          string list of members
        =============== =======================
    """
    Single_Scan = 1
    Accumulate = 2
    Kinetics = 3
    Fast_Kinetics = 4
    Run_till_abort = 5


    @classmethod
    def names(cls):
        return [name for name, member in cls.__members__.items()]


class DAQ_2DViewer_AndorCCD(DAQ_Viewer_base):
    """
        Base class for Andor CCD camera and Shamrock spectrometer


        =============== ==================
        **Attributes**   **Type**

        =============== ==================

        See Also
        --------
        utility_classes.DAQ_Viewer_base
    """
    callback_signal = QtCore.pyqtSignal()
    hardware_averaging = True #will use the accumulate acquisition mode if averaging is neccessary
    params = comon_parameters+[
        {'title': 'Dll library:', 'name': 'andor_lib', 'type': 'browsepath', 'value': str(libpath)},
        
        {'title': 'Camera Settings:', 'name': 'camera_settings', 'type': 'group', 'expanded': True, 'children': [
            {'title': 'Camera Models:', 'name': 'camera_model', 'type': 'list',
                'values': [f"{cam['model']}-{cam['serial']}" for cam in camera_list]},

            {'title': 'Readout Modes:', 'name': 'readout', 'type': 'list', 'values': Andor_Camera_ReadOut.names()[0:-1],
                                            'value': 'FullVertBinning'},
            {'title': 'Readout Settings:', 'name': 'readout_settings', 'type': 'group', 'children':[

                {'title': 'single Track Settings:', 'name': 'st_settings', 'type': 'group', 'visible': False, 'children':[
                    {'title': 'Center pixel:', 'name': 'st_center', 'type': 'int', 'value': 1 , 'default':1, 'min':1},
                    {'title': 'Height:', 'name': 'st_height', 'type': 'int', 'value': 1 , 'default':1, 'min':1},
                ]},    
                {'title': 'Multi Track Settings:', 'name': 'mt_settings', 'type': 'group', 'visible': False, 'children':[
                    {'title': 'Ntrack:', 'name': 'mt_N', 'type': 'int', 'value': 1 , 'default':1, 'min':1},
                    {'title': 'Height:', 'name': 'mt_height', 'type': 'int', 'value': 1 , 'default':1, 'min':1},
                    {'title': 'Offset:', 'name': 'mt_offset', 'type': 'int', 'value': 1 , 'default':1, 'min':0},
                    {'title': 'Bottom:', 'name': 'mt_bottom', 'type': 'int', 'value': 1 , 'default':1, 'min':0, 'readonly': True},
                    {'title': 'Gap:', 'name': 'mt_gap', 'type': 'int', 'value': 1 , 'default':1, 'min':0, 'readonly': True},
                ]},
                {'title': 'Image Settings:', 'name': 'image_settings', 'type': 'group', 'visible': False, 'children':[
                    {'title': 'Binning along x:', 'name': 'bin_x', 'type': 'int', 'value': 1, 'default': 1, 'min': 1},
                    {'title': 'Binning along y:', 'name': 'bin_y', 'type': 'int', 'value': 1, 'default': 1, 'min': 1},
                    {'title': 'Start x:', 'name': 'im_startx', 'type': 'int', 'value': 1 , 'default':1, 'min':0},
                    {'title': 'End x:', 'name': 'im_endx', 'type': 'int', 'value': 1024 , 'default':1024, 'min':0},
                    {'title': 'Start y:', 'name': 'im_starty', 'type': 'int', 'value': 1 , 'default':1, 'min':1},
                    {'title': 'End y:', 'name': 'im_endy', 'type': 'int', 'value': 256, 'default':256, 'min':1,},
                    ]},   
            ]},            
            {'title': 'Exposure (ms):', 'name': 'exposure', 'type': 'float', 'value': 0.01 , 'default':0.01, 'min': 0},
            
            {'title': 'Image size:', 'name': 'image_size', 'type': 'group', 'children':[
                {'title': 'Nx:', 'name': 'Nx', 'type': 'int', 'value': 0, 'default':0 , 'readonly': True},
                {'title': 'Ny:', 'name': 'Ny', 'type': 'int', 'value': 0 , 'default':0 , 'readonly': True},
                ]},
            
            {'title': 'Shutter Settings:', 'name': 'shutter', 'type': 'group', 'children':[
                {'title': 'Open Shutter on:', 'name': 'shutter_type', 'type': 'list', 'value': 'high', 'values': ['low', 'high']},
                {'title': 'Shutter mode:', 'name': 'shutter_mode', 'type': 'list', 'value': 'Auto', 'values': ['Auto', 'Always Opened', 'Always Closed', ]},
                {'title': 'Closing time (ms):', 'name': 'shutter_closing_time', 'type': 'int', 'value': 0, 'tip': 'millisecs it takes to close'},
                {'title': 'Opening time (ms):', 'name': 'shutter_opening_time', 'type': 'int', 'value': 10, 'tip': 'millisecs it takes to open'},
                ]},
            {'title': 'Temperature Settings:', 'name': 'temperature_settings', 'type': 'group', 'children': [
                {'title': 'Set Point:', 'name': 'set_point', 'type': 'float', 'value': -60, 'default': -60},
                {'title': 'Current value:', 'name': 'current_value', 'type': 'float', 'value': 0, 'default': 0,
                 'readonly': True},
                {'title': 'Locked:', 'name': 'locked', 'type': 'led', 'value': False, 'default': False,
                 'readonly': True},
            ]},
        ]},
        ]

    def __init__(self, parent=None, params_state=None):

        super().__init__(parent, params_state)  # initialize base class with commom attributes and methods

        self.x_axis = None
        self.y_axis = None
        self.camera_controller = None
        self.data = None
        self.CCDSIZEX, self.CCDSIZEY = (None, None)
        self.data_pointer = None
        self.camera_done = False
        self.acquirred_image = None
        self.callback_thread = None
        self.Naverage = None
        self.data_shape = None  # 'Data2D' if sizey != 1 else 'Data1D'

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
                self.camera_controller.SetTemperature(param.value())

            elif param.name() == 'readout' or param.name() in iter_children(self.settings.child('camera_settings', 'readout_settings')):
                self.update_read_mode()
                
            elif param.name() == 'exposure':
                self.camera_controller.SetExposureTime(self.settings.child('camera_settings', 'exposure').value() / 1000) #temp should be in s
                (err, timings) = self.camera_controller.GetAcquisitionTimings()
                self.settings.child('camera_settings', 'exposure').setValue(timings['exposure']*1000)
                QtWidgets.QApplication.processEvents()
                self.get_exposure_ms()

            elif param.name() in iter_children(self.settings.child('camera_settings', 'shutter'), []):
                self.set_shutter()

            elif param.name() in iter_children(self.settings.child('camera_settings', 'readout_settings', 'image_settings')):
                if self.settings.child('camera_settings', 'readout').value() == 'Image':
                    self.set_image_area()

            pass


        except Exception as e:
            self.emit_status(ThreadCommand('Update_Status', [str(e), 'log']))

    def emit_data(self):
        """
            Fonction used to emit data obtained by callback.

            See Also
            --------
            daq_utils.ThreadCommand
        """
        try:
            self.ind_grabbed += 1
            sizey = self.settings.child('camera_settings', 'image_size', 'Ny').value()
            sizex = self.settings.child('camera_settings', 'image_size', 'Nx').value()
            self.camera_controller.GetAcquiredDataNumpy(self.data_pointer, sizex * sizey)
            self.data_grabed_signal.emit([DataFromPlugins(name='Camera',
                                                          data=[np.squeeze(
                                                              self.data.reshape((sizey, sizex)).astype(np.float))],
                                                          dim=self.data_shape)])
            QtWidgets.QApplication.processEvents()  # here to be sure the timeevents are executed even if in continuous grab mode

        except Exception as e:
            self.emit_status(ThreadCommand('Update_Status', [str(e), 'log']))


    def update_read_mode(self):
        read_mode = Andor_Camera_ReadOut[self.settings.child('camera_settings', 'readout').value()].value
        err = self.camera_controller.SetReadMode(read_mode)
        if err != 'DRV_SUCCESS':
            self.emit_status(ThreadCommand('Update_Status',[err,'log']))
        else:
            self.settings.child('camera_settings', 'readout_settings').show()
            if read_mode == 0:#FVB:
                self.settings.child('camera_settings', 'readout_settings').hide()
                self.settings.child('camera_settings','image_size','Nx').setValue(self.CCDSIZEX)
                self.settings.child('camera_settings','image_size','Ny').setValue(1)


            elif read_mode == 3: #single track
                self.settings.child('camera_settings', 'readout_settings','mt_settings').hide()
                self.settings.child('camera_settings', 'readout_settings','st_settings').show()
                self.settings.child('camera_settings', 'readout_settings','image_settings').hide()

                err = self.set_single_track_area()

            elif read_mode == 1: #multitrack
                self.settings.child('camera_settings', 'readout_settings','mt_settings').show()
                self.settings.child('camera_settings', 'readout_settings','st_settings').hide()
                self.settings.child('camera_settings', 'readout_settings','image_settings').hide()

                err = self.set_multi_track_area()



            elif read_mode == 2: #random
                err = 'Random mode not implemented yet'
                
            elif read_mode == 4: #image
                self.settings.child('camera_settings', 'readout_settings','mt_settings').hide()
                self.settings.child('camera_settings', 'readout_settings','st_settings').hide()
                self.settings.child('camera_settings', 'readout_settings','image_settings').show()

                self.set_image_area()

                
            elif read_mode == 5: #croped
                err = 'Croped mode not implemented yet'
            self.emit_status(ThreadCommand('Update_Status',[err,'log']))
            
            (err, timings) = self.camera_controller.GetAcquisitionTimings()
            self.settings.child('camera_settings', 'exposure').setValue(timings['exposure']*1000)

            self.x_axis = self.get_xaxis()
            self.y_axis = self.get_yaxis()

    def set_multi_track_area(self):

        N = self.settings.child('camera_settings', 'readout_settings', 'mt_settings', 'mt_N').value()
        height = self.settings.child('camera_settings', 'readout_settings', 'mt_settings', 'mt_height').value()
        offset = self.settings.child('camera_settings', 'readout_settings', 'mt_settings', 'mt_offset').value()
        (err, bottom, gap) = self.camera_controller.SetMultiTrack(N, height, offset)
        self.settings.child('camera_settings', 'readout_settings', 'mt_settings', 'mt_bottom').setValue(bottom)
        self.settings.child('camera_settings', 'readout_settings', 'mt_settings', 'mt_gap').setValue(gap)
        if err == 'DRV_SUCCESS':
            self.settings.child('camera_settings', 'image_size', 'Nx').setValue(self.CCDSIZEX)
            self.settings.child('camera_settings', 'image_size', 'Ny').setValue(N)
        return err

    def set_single_track_area(self):
        center = self.settings.child('camera_settings', 'readout_settings', 'st_settings', 'st_center').value()
        height = self.settings.child('camera_settings', 'readout_settings', 'st_settings', 'st_height').value()
        err = self.camera_controller.SetSingleTrack(center, height)
        if err == 'DRV_SUCCESS':
            self.settings.child('camera_settings', 'image_size', 'Nx').setValue(self.CCDSIZEX)
            self.settings.child('camera_settings', 'image_size', 'Ny').setValue(1)

        return err


    def set_image_area(self):

        binx = self.settings.child('camera_settings', 'readout_settings', 'image_settings', 'bin_x').value()
        biny = self.settings.child('camera_settings', 'readout_settings','image_settings', 'bin_y').value()
        startx = self.settings.child('camera_settings', 'readout_settings', 'image_settings', 'im_startx').value()
        endx = self.settings.child('camera_settings', 'readout_settings', 'image_settings', 'im_endx').value()
        starty = self.settings.child('camera_settings', 'readout_settings', 'image_settings', 'im_starty').value()
        endy = self.settings.child('camera_settings', 'readout_settings', 'image_settings', 'im_endy').value()
        err = self.camera_controller.SetImage(binx, biny, startx, endx, starty, endy)
        if err == 'DRV_SUCCESS':
            self.settings.child('camera_settings', 'image_size', 'Nx').setValue(int((endx-startx+1)/binx))
            self.settings.child('camera_settings', 'image_size', 'Ny').setValue(int((endy-starty+1)/biny))

        return err

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
            self.emit_status(ThreadCommand('show_splash', ["Initialising Andor CCD Camera "]))
            if self.settings.child(('controller_status')).value() == "Slave":
                if controller is None:
                    raise Exception('no controller has been defined externally while this detector is a slave one')
                else:
                    self.camera_controller = controller
            else:
                self.camera_controller = sdk2.AndorSDK()

            self.emit_status(ThreadCommand('show_splash', ["Set/Get Camera's settings"]))
            self.ini_camera()

            # %%%%%%% init axes from image
            self.x_axis = self.get_xaxis()
            self.y_axis = self.get_yaxis()
            self.status.x_axis = self.x_axis
            self.status.y_axis = self.y_axis
            self.status.initialized = True
            self.status.controller = self.camera_controller
            self.emit_status(ThreadCommand('close_splash'))
            return self.status

        except Exception as e:
            self.status.info = str(e)
            self.status.initialized = False
            self.emit_status(ThreadCommand('close_splash'))
            return self.status

    def get_ROI_size_x(self):
        self.CCDSIZEX, self.CCDSIZEY = self.camera_controller.GetDetector()
        return self.CCDSIZEX

    def get_pixel_size(self):
        err, (width, height) = self.camera_controller.GetPixelSize()
        if err == 'DRV_SUCCESS':
            return width, height
        else:
            pass
        return 0., 0.

    def ini_camera(self):
        #  %%%%%% Get image size and current binning
        # get info from camera
        model_param = self.settings.child('camera_settings', 'camera_model')
        cam_index = model_param.opts['limits'].index(model_param.value())
        self.camera_controller.SetCurrentCamera(camera_list[cam_index]['handle'])

        self.CCDSIZEX, self.CCDSIZEY = self.camera_controller.GetDetector()
        self.settings.child('camera_settings', 'readout_settings',
                            'st_settings', 'st_center').setLimits((1, self.CCDSIZEY))
        self.settings.child('camera_settings', 'readout_settings',
                            'st_settings', 'st_height').setLimits((1, self.CCDSIZEY))
        self.settings.child('camera_settings', 'readout_settings', 'image_settings', 'im_endy').setValue(self.CCDSIZEY)
        self.settings.child('camera_settings', 'readout_settings', 'image_settings',
                            'im_endy').setOpts(max=self.CCDSIZEY, default=self.CCDSIZEY)
        self.settings.child('camera_settings', 'readout_settings', 'image_settings', 'im_endx').setValue(self.CCDSIZEX)
        self.settings.child('camera_settings', 'readout_settings', 'image_settings',
                            'im_endx').setOpts(max=self.CCDSIZEX, default=self.CCDSIZEX)

        # get max exposure range
        err, maxexpo = self.camera_controller.GetMaximumExposure()
        if err == 'DRV_SUCCESS':
            self.settings.child('camera_settings', 'exposure').setLimits((0, maxexpo))

        # set default read mode (full vertical binning)
        self.update_read_mode()

        # %%%%%%% Set and Get temperature from camera
        # get temperature range
        (err, temp_range) = self.camera_controller.GetTemperatureRange()
        if err == "DRV_SUCCESS":
            self.settings.child('camera_settings', 'temperature_settings', 'set_point').setLimits(
                (temp_range[0], temp_range[1]))


        self.set_shutter()

        if not self.camera_controller.IsCoolerOn():  # gets 0 or 1
            self.camera_controller.CoolerON()

        self.camera_controller.SetTemperature(
            self.settings.child('camera_settings', 'temperature_settings', 'set_point').value())
        locked_status, temp = self.camera_controller.GetTemperature()
        self.settings.child('camera_settings', 'temperature_settings', 'current_value').setValue(temp)
        self.settings.child('camera_settings', 'temperature_settings', 'locked').setValue(
            locked_status == 'DRV_TEMP_STABILIZED')
        # set timer to update temperature info from controller
        self.timer = self.startTimer(2000)  # Timer event fired every 2s

        callback = AndorCallback(self.camera_controller.WaitForAcquisition)
        self.callback_thread = QtCore.QThread()
        callback.moveToThread(self.callback_thread)
        callback.data_sig.connect(self.emit_data)  # when the wait for acquisition returns (with data taken), emit_data will be fired

        self.callback_signal.connect(callback.wait_for_acquisition)
        self.callback_thread.callback = callback
        self.callback_thread.start()


    def set_shutter(self):
        typ = self.settings.child('camera_settings', 'shutter', 'shutter_type').opts['limits'].index(
                                        self.settings.child('camera_settings', 'shutter', 'shutter_type').value())
        mode = self.settings.child('camera_settings', 'shutter', 'shutter_mode').opts['limits'].index(
                                        self.settings.child('camera_settings', 'shutter', 'shutter_mode').value())

        self.camera_controller.SetShutter(typ, mode, self.settings.child('camera_settings', 'shutter', 'shutter_closing_time').value(),
                                          self.settings.child('camera_settings', 'shutter', 'shutter_opening_time').value())

    def timerEvent(self, event):
        """

        """
        locked_status, temp = self.camera_controller.GetTemperature()
        self.settings.child('camera_settings', 'temperature_settings', 'current_value').setValue(temp)
        self.settings.child('camera_settings', 'temperature_settings', 'locked').setValue(
            locked_status == 'DRV_TEMP_STABILIZED')

    def close(self):
        """

        """

        err, temp = self.camera_controller.GetTemperature()
        print(temp)
        if temp < -20.:
            print(
                "Camera temperature is still at {:d}°C. Closing it now may damage it! The cooling will be maintained "
                "while shutting down camera. Keep it power plugged!!!".format(
                    temp))
            self.camera_controller.SetCoolerMode(1)
        self.timer.stop()
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
            raise(Exception('controller not defined'))
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

    def prepare_data(self):
        sizex = self.settings.child('camera_settings', 'image_size', 'Nx').value()
        sizey = self.settings.child('camera_settings', 'image_size', 'Ny').value()

        # %%%%%% Initialize data: self.data for the memory to store new data and self.data_average to store the average data
        image_size = sizex * sizey
        self.data = np.zeros((image_size,), dtype=np.long)
        self.data_pointer = self.data.ctypes.data_as(ctypes.c_void_p)

        data_shape = 'Data2D' if sizey != 1 else 'Data1D'
        if data_shape != self.data_shape:
            self.data_shape = data_shape
            # init the viewers
            self.data_grabed_signal_temp.emit([DataFromPlugins(name='Camera ',
                                                               data=[np.squeeze(
                                                                   self.data.reshape((sizey, sizex)).astype(np.float))],
                                                               dim=self.data_shape)])

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

            self.ind_grabbed = 0  # to keep track of the current image in the average
            self.Naverage = Naverage  #

            self.prepare_data()
            if Naverage == 1:
                self.camera_controller.SetAcquisitionMode(1)
            else:
                self.camera_controller.SetAcquisitionMode(2)
                self.camera_controller.SetNumberAccumulations(Naverage)

            self.camera_controller.SetExposureTime(
                self.settings.child('camera_settings', 'exposure').value() / 1000)  # temp should be in s
            (err, timings) = self.camera_controller.GetAcquisitionTimings()
            self.settings.child('camera_settings', 'exposure').setValue(timings['exposure'] * 1000)
            # %%%%% Start acquisition with the given exposure in ms, in "1d" or "2d" mode
            self.camera_controller.StartAcquisition()
            self.callback_signal.emit()  # will trigger the waitfor acquisition

        except Exception as e:
            self.emit_status(ThreadCommand('Update_Status', [str(e), "log"]))

    def stop(self):
        """
            stop the camera's actions.
        """
        try:
            self.camera_controller.CancelWait()  # first cancel the waitacquistion (if any)
            QtWidgets.QApplication.processEvents()
            self.camera_controller.AbortAcquisition()  # abort the camera actions

        except:
            pass
        return ""


class AndorCallback(QtCore.QObject):
    """

    """
    data_sig = QtCore.pyqtSignal()

    def __init__(self, wait_fn):
        super(AndorCallback, self).__init__()
        self.wait_fn = wait_fn

    def wait_for_acquisition(self):
        err = self.wait_fn()

        if err != 'DRV_NO_NEW_DATA':  # will be returned if the main thread called CancelWait
            self.data_sig.emit()
