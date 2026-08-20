import numpy as np
import cv2
from qtpy import QtWidgets

from pymodaq_utils.logger import set_logger, get_module_name
from pymodaq_utils.utils import ThreadCommand

from pymodaq_gui.parameter import utils as putils

from pymodaq.utils.data import Axis, DataFromPlugins, DataToExport
from pymodaq.control_modules.viewer_utility_classes import main, comon_parameters

from pymodaq_plugins_andor.daq_viewer_plugins.plugins_2D.daq_2Dviewer_AndorCCDPll import DAQ_2DViewer_AndorCCDPll
from pymodaq_plugins_andor.daq_move_plugins.daq_move_ShamrockPll import DAQ_Move_ShamrockPll

from pymodaq_plugins_andor.hardware.shamrockccd_utils import ShamrockCCDCompo

logger = set_logger(get_module_name(__file__))


class DAQ_1DViewer_ShamrockCCDPll(DAQ_2DViewer_AndorCCDPll):
    """
        =============== ==================

        =============== ==================

        See Also
        --------
        utility_classes.DAQ_Viewer_base
    """
    params_shamrock = DAQ_Move_ShamrockPll.params
    putils.get_param_dict_from_name(params_shamrock, 'andor_lib', pop=True)

    d = putils.get_param_dict_from_name(params_shamrock, 'spectro_wl')
    if d is not None:
        d['readonly'] = False
    d = putils.get_param_dict_from_name(params_shamrock, 'flip_wavelength')
    if d is not None:
        d['visible'] = True

    params = DAQ_2DViewer_AndorCCDPll.params + [
        {'title': 'Get Calibration:', 'name': 'get_calib', 'type': 'bool_push', 'value': False, 'label': 'Update!'},
        {'title': 'Shamrock Settings', 'name': 'sham_settings', 'type': 'group', 'children': params_shamrock},
    ]

    def ini_attributes(self):
        self.controller: ShamrockCCDCompo = None
        self.shamrock_controller: DAQ_Move_ShamrockPll = None

        self.x_axis: Axis = None
        self.is_calibrated = False

        super().ini_attributes()

    def commit_settings(self, param):

        super().commit_settings(param)
        if param.name() == 'flip_wavelength':
            self.get_xaxis()
        elif 'sham_settings' in putils.get_param_path(param):
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
            putils.iter_children(self.settings.child('readout_settings')):
            self.get_xaxis()
        elif param.name() == 'get_calib':
            if param.value():
                self.get_xaxis()
                param.setValue(False)

    def ini_detector(self, controller=None):


        ind_camera = self.settings.child('serial_number').opts['limits'].index(self.settings['serial_number'])
        ind_spectro = self.settings.child('sham_settings',
                                          'spectro_sn').opts['limits'].index(self.settings['sham_settings', 'spectro_sn'])
        if self.is_master:
            self.controller = ShamrockCCDCompo(cam_idx=ind_camera, spec_idx=ind_spectro)

        cam_status, cam_init = super().ini_detector(self.controller)
        QtWidgets.QApplication.processEvents()

        self.shamrock_controller = DAQ_Move_ShamrockPll(None, self.settings.child('sham_settings').saveState())
        self.shamrock_controller.settings = self.settings.child('sham_settings')
        self.settings.child('sham_settings','controller').hide()
        self.shamrock_controller.emit_status = self.emit_status
        sham_status, sham_init = self.shamrock_controller.ini_stage(self.controller.shamrock)

        QtWidgets.QApplication.processEvents()

        initialized = sham_init and cam_init

        self.setCalibration()
        return sham_status + cam_status, initialized

    def setCalibration(self):
        #setNpixels
        width = self.controller.get_pixel_size()[0]
        self.shamrock_controller.controller.set_number_pixels(self.ccdsize_x)
        self.shamrock_controller.controller.set_pixel_width(width)

        self.settings.child('sham_settings',
                            'spectro_settings',
                            'spectro_wl').setValue(self.shamrock_controller.controller.get_wavelength()*1e9)
        self.x_axis = self.get_xaxis()

    def get_xaxis(self):
        """
            Obtain the horizontal axis of the image.

            Returns
            -------
            1D numpy array
                Contains a vector of integer corresponding to the horizontal camera pixels.
        """

        if self.shamrock_controller is None or np.abs(self.settings.child('sham_settings',
                                                                          'spectro_settings',
                                                                          'spectro_wl').value()) < 1e-3:
            nx = self.ccdsize_x
            calib = np.linspace(0, nx, nx-1)
            self.x_axis = Axis(data=calib, label='Wavelength', units='nm')
        else:
            calib = self.shamrock_controller.controller.get_calibration()*1e9

            if (calib.astype('int') != 0).all():  # check if calib values are equal to zero
                if self.settings.child('sham_settings', 'spectro_settings', 'flip_wavelength').value():
                    calib = calib[::-1]

            else:
                self.settings.child('sham_settings', 'spectro_settings', 'flip_wavelength').setValue(False)
                self.emit_status(ThreadCommand('Update_Status', ['Impossible to flip wavelength', "log"]))

            self.x_axis = Axis(data=calib, label='Wavelength', units='nm')
        return self.x_axis

    def stop(self):
        if self.controller is not None:
            super().stop()
        if self.shamrock_controller is not None:
            self.shamrock_controller.stop_motion()

    def close(self):
        self.stop()
        if self.shamrock_controller is not None:
            self.shamrock_controller.close()
        super().close()

    def grab_data(self, Naverage=1, **kwargs):
        if not self.is_calibrated:
            self.get_xaxis()
        super().grab_data(Naverage, **kwargs)

    def emit_data(self, frame: np.ndarray=None):
        """
            Overloaded function from camera_base_pylablib
            Function used to emit data obtained by callback.

                Parameter
                ---------
                status: bool
                    If True a frame is available, If False, a Timeout occurred while waiting for the frame

                See Also
                --------
                daq_utils.ThreadCommand
        """
        try:
            # Get  data from buffer
            if frame is None:
                frame = self.controller.read_newest_image()
            # Emit the frame.
            if frame is not None:
                conversion_str = self.settings['color_conversion']
                if conversion_str != "None":
                    for ind_average in range(frame.shape[0]):
                        for ind_frame in range(frame.shape[1]):
                            if ind_frame == 0 and ind_average == 0:
                                new_frame = cv2.cvtColor(frame[ind_average, ind_frame, ...],
                                                         getattr(cv2, f'COLOR_{conversion_str}'))
                                shape = [frame.shape[0], frame.shape[1]] + list(new_frame.shape)
                                out_frames = np.zeros(shape, dtype=new_frame.dtype)
                                out_frames[ind_average, ind_frame, ...] = new_frame
                            else:
                                cv2.cvtColor(frame[ind_average, ind_frame, ...],
                                             getattr(cv2, f'COLOR_{conversion_str}'),
                                             out_frames[ind_average, ind_frame, ...])
                else:
                    out_frames = frame
                if self.Naverage > 1:
                    out_frames = np.sum(out_frames, axis=0) / self.Naverage
                else:
                    out_frames = out_frames[0, ...]

                if self.n_frames > 1:
                    pass
                    # todo handle chunks of frames in ND data
                else:
                    out_frames = out_frames[0, ...]

                labels = ['Intensity']
                data_arrays = [out_frames]

                if self.data_shape == 'Data1D':
                    data_name = 'Spectrum'
                    data_arrays = np.squeeze(data_arrays)
                else:
                    data_name = 'Camera'

                self.dte_signal.emit(
                    DataToExport('Spectrometer',
                                 data=[DataFromPlugins(name=data_name,
                                                       data=data_arrays,
                                                       dim=self.data_shape,
                                                       labels=labels,
                                                       axes=[self.x_axis])]))
            if self.settings.child('timing_opts', 'fps_on').value():
                self.update_fps()

            # To make sure that timed events are executed in continuous grab mode
            QtWidgets.QApplication.processEvents()

        except Exception as e:
            self.emit_status(ThreadCommand('Update_Status', [str(e), 'log']))


if __name__ == '__main__':
    main(__file__, True)
