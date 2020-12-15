from ..plugins_2D.daq_2Dviewer_AndorCCD import DAQ_2DViewer_AndorCCD
from ...daq_move_plugins.daq_move_Shamrock import DAQ_Move_Shamrock
import numpy as np
from pymodaq.daq_utils.daq_utils import ThreadCommand, find_dict_in_list_from_key_val, Axis
from pymodaq.daq_utils.parameter import utils as putils
from PyQt5 import QtWidgets


class DAQ_1DViewer_ShamrockCCD(DAQ_2DViewer_AndorCCD, DAQ_Move_Shamrock):
    """
        =============== ==================

        =============== ==================

        See Also
        --------
        utility_classes.DAQ_Viewer_base
    """

    param_camera = DAQ_2DViewer_AndorCCD.params
    params_shamrock = DAQ_Move_Shamrock.params
    putils.get_param_dict_from_name(params_shamrock, 'andor_lib', pop=True)

    d = putils.get_param_dict_from_name(params_shamrock, 'spectro_wl')
    if d is not None:
        d['readonly'] = False
    d = putils.get_param_dict_from_name(params_shamrock, 'flip_wavelength')
    if d is not None:
        d['visible'] = True

    params = [{'title': 'Get Calibration:', 'name': 'get_calib', 'type': 'bool_push', 'value': False,
              'label': 'Update!'}] + param_camera + params_shamrock



    def __init__(self, parent=None, params_state=None):

        DAQ_2DViewer_AndorCCD.__init__(self, parent, params_state)
        DAQ_Move_Shamrock.__init__(self, parent, params_state)

        self.camera_controller = None  # this will be the controller attribute of the  DAQ_2DViewer_AndorCCD instance
        self.shamrock_controller = None  # this will be the controller attribute of the  DAQ_Move_Shamrock instance
        # both plugins don't have the generic controller name 'controller' but specific one for this reason

        self.x_axis = None
        self.is_calibrated = False

    def commit_settings(self, param):

        if param.name() == 'flip_wavelength':
            self.get_xaxis()
        elif 'camera_settings' in putils.get_param_path(param):
            DAQ_2DViewer_AndorCCD.commit_settings(self, param)
        elif 'spectro_settings' in putils.get_param_path(param):
            DAQ_Move_Shamrock.commit_settings(self, param)
        QtWidgets.QApplication.processEvents()
        if param.name() == 'spectro_wl':
            self.is_calibrated = False
            self.get_xaxis()
        elif param.name() == 'zero_order':
            self.is_calibrated = False
        elif param.name() == 'flip_wavelength':
            self.get_xaxis()
        elif param.name() == 'readout' or param.name() in \
            putils.iter_children(self.settings.child('camera_settings', 'readout_settings')):
            self.get_xaxis()
        elif param.name() == 'get_calib':
            if param.value():
                self.get_xaxis()
                param.setValue(False)

    def ini_detector(self, controller=None):
        try:
            status_shamrock = DAQ_Move_Shamrock.ini_stage(self, controller)
            QtWidgets.QApplication.processEvents()
            # if status_shamrock.initialized:
            #     self.move_Home()

            status_camera = DAQ_2DViewer_AndorCCD.ini_detector(self, controller)
            QtWidgets.QApplication.processEvents()

            self.status.initialized = status_shamrock.initialized and status_camera.initialized

            self.setCalibration()
            return self.status

        except Exception as e:
            self.status.info = str(e)
            self.status.initialized = False
            self.emit_status(ThreadCommand('close_splash'))
            return self.status

    def setCalibration(self):
        #setNpixels
        width, height = self.get_pixel_size()
        err = self.shamrock_controller.SetNumberPixelsSR(0, self.get_ROI_size_x())
        err = self.shamrock_controller.SetPixelWidthSR(0, width)

        self.get_wavelength()
        self.x_axis = self.get_xaxis()


    def getCalibration(self):

        if self.shamrock_controller is not None:
            (err, calib) = self.shamrock_controller.GetCalibrationSR(0, self.get_ROI_size_x())
            if err != "SHAMROCK_SUCCESS":
                raise Exception(err)

            calib = np.array(calib)
        else:
            raise(Exception('controller not defined'))
        self.is_calibrated = True
        return calib

    def get_xaxis(self):
        """
            Obtain the horizontal axis of the image.

            Returns
            -------
            1D numpy array
                Contains a vector of integer corresponding to the horizontal camera pixels.
        """

        if np.abs(self.settings.child('spectro_settings', 'spectro_wl').value()) < 1e-3:
            DAQ_2DViewer_AndorCCD.get_xaxis(self)
        else:
            calib = self.getCalibration()

            if (calib.astype('int') != 0).all():  # check if calib values are equal to zero
                if self.settings.child('spectro_settings', 'flip_wavelength').value():
                    calib = calib[::-1]

            else:
                self.settings.child('spectro_settings', 'flip_wavelength').setValue(False)
                self.emit_status(ThreadCommand('Update_Status', ['Impossible to flip wavelength', "log"]))

            self.x_axis = Axis(data=calib, label='Wavelength (nm)')
            self.emit_x_axis()
        return self.x_axis


    def get_exposure_ms(self):
        #for compatibility with PyMoDAQ Spectro module
        self.emit_status(ThreadCommand('exposure_ms', [self.settings.child('camera_settings', 'exposure').value()]))

    def set_exposure_ms(self, exposure):
        self.settings.child('camera_settings', 'exposure').setValue(exposure)
        QtWidgets.QApplication.processEvents()
        self.emit_status(ThreadCommand('exposure_ms', [self.settings.child('camera_settings', 'exposure').value()]))

    def stop(self):
        DAQ_2DViewer_AndorCCD.stop(self)

    def close(self):
        DAQ_2DViewer_AndorCCD.stop(self)
        DAQ_Move_Shamrock.stop(self)

    def grab_data(self, Naverage=1, **kwargs):
        DAQ_2DViewer_AndorCCD.grab_data(self, Naverage, **kwargs)
        if not self.is_calibrated:
            self.get_xaxis()
