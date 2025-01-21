import numpy as np
from qtpy import QtWidgets

from pymodaq_utils.logger import set_logger, get_module_name
from pymodaq_utils.utils import ThreadCommand, find_dict_in_list_from_key_val

from pymodaq_gui.parameter import utils as putils

from pymodaq.utils.data import Axis, DataFromPlugins, DataToExport
from pymodaq.control_modules.viewer_utility_classes import main, DAQ_Viewer_base, comon_parameters

from pymodaq_plugins_andor.daq_viewer_plugins.plugins_2D.daq_2Dviewer_AndorCCD import DAQ_2DViewer_AndorCCD
from pymodaq_plugins_andor.daq_move_plugins.daq_move_Shamrock import DAQ_Move_Shamrock

logger = set_logger(get_module_name(__file__))


class DAQ_1DViewer_ShamrockCCDComposition(DAQ_2DViewer_AndorCCD):
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
              'label': 'Update!'},] + param_camera + [
              {'title': 'Shamrock Settings:', 'name': 'sham_settings', 'type': 'group', 'children': params_shamrock},
    ]


    def ini_attributes(self):
        self.controller: DAQ_2DViewer_AndorCCD = None
        self.shamrock_controller: DAQ_Move_Shamrock = None

        self.x_axis: Axis = None
        self.is_calibrated = False

        super().ini_attributes()

    def commit_settings(self, param):

        if param.name() == 'flip_wavelength':
            self.get_xaxis()
        elif 'camera_settings' in putils.get_param_path(param):
            super().commit_settings(param)
        elif 'spectro_settings' in putils.get_param_path(param):
            self.shamrock_controller.commit_settings(param)
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
        cam_status, cam_init = super().ini_detector(controller)
        QtWidgets.QApplication.processEvents()

        self.shamrock_controller = DAQ_Move_Shamrock(None, self.settings.child('sham_settings').saveState())
        self.shamrock_controller.settings = self.settings.child('sham_settings')
        self.shamrock_controller.emit_status = self.emit_status
        sham_status, sham_init = self.shamrock_controller.ini_stage(controller)

        QtWidgets.QApplication.processEvents()


        initialized = sham_init and cam_init

        self.setCalibration()
        return sham_status + cam_status, initialized

    def setCalibration(self):
        #setNpixels
        width, height = self.get_pixel_size()
        err = self.shamrock_controller.controller.SetNumberPixelsSR(0, self.get_ROI_size_x())
        err = self.shamrock_controller.controller.SetPixelWidthSR(0, width)

        self.shamrock_controller.get_wavelength()
        self.x_axis = self.get_xaxis()


    def getCalibration(self):

        if self.shamrock_controller is not None and self.shamrock_controller.controller is not None:
            (err, calib) = self.shamrock_controller.controller.GetCalibrationSR(0, self.get_ROI_size_x())
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

        if self.shamrock_controller is None or np.abs(self.settings.child('sham_settings', 'spectro_settings', 'spectro_wl').value()) < 1e-3:
            nx = self.get_ROI_size_x()
            calib = np.linspace(0, nx, nx-1)
            self.x_axis = Axis(data=calib, label='Wavelength (nm)')
        else:
            calib = self.getCalibration()

            if (calib.astype('int') != 0).all():  # check if calib values are equal to zero
                if self.settings.child('sham_settings', 'spectro_settings', 'flip_wavelength').value():
                    calib = calib[::-1]

            else:
                self.settings.child('sham_settings', 'spectro_settings', 'flip_wavelength').setValue(False)
                #self.emit_status(ThreadCommand('Update_Status', ['Impossible to flip wavelength', "log"]))

            self.x_axis = Axis(data=calib, label='Wavelength (nm)')
        return self.x_axis

    def get_exposure_ms(self):
        #for compatibility with PyMoDAQ Spectro module
        self.emit_status(ThreadCommand('exposure_ms', [self.settings.child('camera_settings', 'exposure').value()]))

    def set_exposure_ms(self, exposure):
        self.settings.child('camera_settings', 'exposure').setValue(exposure)
        QtWidgets.QApplication.processEvents()
        self.emit_status(ThreadCommand('exposure_ms', [self.settings.child('camera_settings', 'exposure').value()]))

    def stop(self):
        if self.controller is not None:
            super().stop()
        if self.shamrock_controller is not None:
            self.shamrock_controller.stop()

    def close(self):
        self.stop()
        if self.shamrock_controller is not None:
            self.shamrock_controller.close()
        super().close()

    def grab_data(self, Naverage=1, **kwargs):
        if not self.is_calibrated:
            self.get_xaxis()
        super().grab_data( Naverage, **kwargs)

    def emit_data(self):
        """
            overloadded function from DAQ_2DViewer_AndorCCD
        """
        try:
            self.ind_grabbed += 1
            sizey = self.settings.child('camera_settings', 'image_size', 'Ny').value()
            sizex = self.settings.child('camera_settings', 'image_size', 'Nx').value()
            self.controller.GetAcquiredDataNumpy(self.data_pointer, sizex * sizey)
            self.dte_signal.emit(
                DataToExport('Spectro',
                             data=[
                                 DataFromPlugins(name='Camera',
                                                 data=[np.atleast_1d(np.squeeze(self.data.reshape(
                                                     (sizey, sizex)))).astype(float)],
                                                 dim=self.data_shape,
                                                 axes=[self.x_axis]),
                             ]))
            QtWidgets.QApplication.processEvents()  # here to be sure the timeevents are executed even if in continuous grab mode

        except Exception as e:
            self.emit_status(ThreadCommand('Update_Status', [str(e), 'log']))


if __name__ == '__main__':
    main(__file__, True)
