from ..plugins_2D.daq_2Dviewer_AndorSCMOS import DAQ_2DViewer_AndorSCMOS
from ...daq_move_plugins.daq_move_Shamrock import DAQ_Move_Shamrock

import numpy as np
from pymodaq.utils.daq_utils import ThreadCommand, find_dict_in_list_from_key_val
from pymodaq.utils.data import Axis, DataFromPlugins
from pymodaq.utils.parameter import utils as putils
from qtpy import QtWidgets


class DAQ_1DViewer_ShamrockSCMOS(DAQ_2DViewer_AndorSCMOS, DAQ_Move_Shamrock):
    """
        =============== ==================

        =============== ==================

        See Also
        --------
        utility_classes.DAQ_Viewer_base
    """

    param_camera = DAQ_2DViewer_AndorSCMOS.params
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

        DAQ_2DViewer_AndorSCMOS.__init__(self, parent, params_state)
        DAQ_Move_Shamrock.__init__(self, parent, params_state)

        self.camera_controller = None  # this will be the controller attribute of the  DAQ_2DViewer_AndorSCMOS instance
        self.shamrock_controller = None  # this will be the controller attribute of the  DAQ_Move_Shamrock instance
        # both plugins don't have the generic controller name 'controller' but specific one for this reason

        self.x_axis = None
        self.is_calibrated = False

    def commit_settings(self, param):

        if param.name() == 'flip_wavelength':
            self.get_xaxis()
        elif 'camera_settings' in putils.get_param_path(param):
            DAQ_2DViewer_AndorSCMOS.commit_settings(self, param)
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
        _, shamrock_initialized = DAQ_Move_Shamrock.ini_stage(self, controller)
        QtWidgets.QApplication.processEvents()
        # if status_shamrock.initialized:
        #     self.move_Home()

        _, camera_initialized = DAQ_2DViewer_AndorSCMOS.ini_detector(self, controller)
        QtWidgets.QApplication.processEvents()

        initialized = shamrock_initialized and camera_initialized

        self.setCalibration()
        return '', initialized

    def setCalibration(self):
        #setNpixels
        width = self.get_pixel_size()
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
            DAQ_2DViewer_AndorSCMOS.get_xaxis(self)
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
        DAQ_2DViewer_AndorSCMOS.stop(self)

    def close(self):
        DAQ_2DViewer_AndorSCMOS.stop(self)
        DAQ_Move_Shamrock.stop(self)

    def grab_data(self, Naverage=1, **kwargs):
        if not self.is_calibrated:
            self.get_xaxis()
        DAQ_2DViewer_AndorSCMOS.grab_data(self, Naverage, **kwargs)

    def emit_data(self, buffer_pointer):
        """
            overloadded function from DAQ_2DViewer_AndorScmos
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
                            DataFromPlugins(name=cam_name, data=[self.data], dim=self.data_shape,
                                            x_axis=self.x_axis)])

                    elif self.n_grabed_data < self.Naverage:
                        self.data_grabed_signal_temp.emit([
                            DataFromPlugins(name=cam_name, data=[self.data * self.Naverage / self.n_grabed_data],
                                            dim=self.data_shape,
                                            x_axis=self.x_axis)])
            else:  # in live mode
                if perf_counter() - self.start_time > self.refresh_time_fr / 1000:  # refresh the frame rate every
                    # refresh_time_fr ms
                    self.settings.child('camera_settings',
                                        'frame_rate').setValue(self.n_grabed_frame_rate / (self.refresh_time_fr / 1000))
                    self.start_time = perf_counter()
                    self.n_grabed_frame_rate = 0

                if self.n_grabed_data % self.Naverage == 0:
                    self.data_grabed_signal.emit([
                        DataFromPlugins(name=cam_name, data=[self.data], dim=self.data_shape,
                                        x_axis=self.x_axis)])
                else:
                    if self.n_grabed_data % self.Naverage != 0:
                        n_grabed = self.n_grabed_data % self.Naverage
                    else:
                        n_grabed = self.Naverage
                    self.data_grabed_signal_temp.emit([
                        DataFromPlugins(name=cam_name,
                                        data=[self.data * self.Naverage / n_grabed],
                                        dim=self.data_shape,
                                        x_axis=self.x_axis)])

            self.camera_controller.queue_single_buffer(self.buffers[self.current_buffer])

        except Exception as e:
            logger.exception(str(e))


