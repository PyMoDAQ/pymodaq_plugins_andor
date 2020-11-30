from ..plugins_2D.daq_2Dviewer_AndorCCD import DAQ_2DViewer_AndorCCD
from ...daq_move_plugins.daq_move_Shamrock import DAQ_Move_Shamrock
import numpy as np
from pymodaq.daq_utils.daq_utils import ThreadCommand, find_dict_in_list_from_key_val, Axis
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
    d, ind = find_dict_in_list_from_key_val(params_shamrock, 'name', 'andor_lib', return_index=True)
    if d is not None:
        params_shamrock.pop(ind)
    #  TODO
    #  Iterate over a flattened list (here the right dict is within a children entry....
    d = find_dict_in_list_from_key_val(params_shamrock, 'name', 'spectro_wl')
    if d is not None:
        d['readonly'] = False
    d = find_dict_in_list_from_key_val(params_shamrock, 'name', 'flip_wavelength')
    if d is not None:
        d['visible'] = True
    params = param_camera + params_shamrock

    def __init__(self, parent=None, params_state=None):

        DAQ_2DViewer_AndorCCD.__init__(self, parent, params_state)
        DAQ_Move_Shamrock.__init__(self, parent, params_state)

        self.x_axis = None
        self.camera_controller = None
        self.shamrock_controller = None

    def commit_settings(self, param):

        DAQ_2DViewer_AndorCCD.commit_settings(self, param)
        DAQ_Move_Shamrock.commit_settings(self, param)

        if param.name() == 'flip_wavelength':
            self.get_xaxis()

    def ini_detector(self, controller=None):
        try:
            status_shamrock = DAQ_Move_Shamrock.ini_stage(self, controller)
            self.shamrock_controller = status_shamrock.controller

            status_camera = DAQ_2DViewer_AndorCCD.ini_detector(self, controller)
            self.camera_controller = status_camera.controller

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
        err, (width, height) = self.camera_controller.GetPixelSize()
        err = self.shamrock_controller.SetNumberPixelsSR(0, self.CCDSIZEX)
        err = self.shamrock_controller.SetPixelWidthSR(0, width)

        self.get_wavelength()
        self.x_axis = self.get_xaxis()


    def getCalibration(self):

        if self.shamrock_controller is not None:
            (err, calib) = self.shamrock_controller.GetCalibrationSR(0, self.CCDSIZEX)
            if err != "SHAMROCK_SUCCESS":
                raise Exception(err)

            calib = np.array(calib)
        else:
            raise(Exception('controller not defined'))
        return calib

    def get_xaxis(self):
        """
            Obtain the horizontal axis of the image.

            Returns
            -------
            1D numpy array
                Contains a vector of integer corresponding to the horizontal camera pixels.
        """
        calib = self.getCalibration()

        if (calib.astype('int') != 0).all():  # check if calib values are equal to zero
            if self.settings.child('spectro_settings', 'flip_wavelength').value():
                calib = calib[::-1]

        else:
            self.settings.child('spectro_settings', 'flip_wavelength').setValue(False)
            self.emit_status(ThreadCommand('Update_Status', ['Impossible to flip wavelength', "log"]))

        self.x_axis = Axis(data=calib, label='Wavelength (nm)')
        return self.x_axis


    def get_exposure_ms(self):
        #for compatibility with PyMoDAQ Spectro module
        self.emit_status(ThreadCommand('exposure_ms', [self.settings.child('camera_settings', 'exposure').value()]))

    def set_exposure_ms(self, exposure):
        self.settings.child('camera_settings', 'exposure').setValue(exposure)
        QtWidgets.QApplication.processEvents()
        self.emit_status(ThreadCommand('exposure_ms', [self.settings.child('camera_settings', 'exposure').value()]))
