from typing import Union, List, Dict
from pymodaq.control_modules.move_utility_classes import (DAQ_Move_base, comon_parameters_fun,
                                                          main, DataActuatorType, DataActuator)

from pymodaq_utils.utils import ThreadCommand  # object used to send info back to the main thread


from pylablib.devices.Andor.Shamrock import ShamrockSpectrograph
from pymodaq_plugins_andor.hardware.shamrock_utils import get_spectrometers

SPEC_NAMES = get_spectrometers()

class DAQ_Move_ShamrockPll(DAQ_Move_base):
    """ Instrument plugin class for the Shamrock series of spectrometers by Andor.
        Uses the pylablib library to communicate with the hardware.

        Attributes:
        -----------
        controller: object
            The particular object that allow the communication with the hardware, in general a python wrapper around the
            hardware library.
        """

    is_multiaxes = False
    _axis_names: Union[List[str], Dict[str, int]] = ['Wavelength']
    _controller_units: Union[str, List[str]] = ['nm']
    _epsilon: Union[float, List[float]] = 0.1
    data_actuator_type = DataActuatorType.DataActuator


    spectro_params = [
        {'title': 'Spectro SN:', 'name': 'spectro_sn', 'type': 'list', 'value': SPEC_NAMES[0],
            'limits': SPEC_NAMES},
        {'title': 'Spectro Settings:', 'name': 'spectro_settings', 'type': 'group', 'expanded': True, 'children': [
            {'title': 'Wavelength (nm):', 'name': 'spectro_wl', 'type': 'float', 'value': 600, 'min': 0},
            {'title': 'Home Wavelength (nm):', 'name': 'spectro_wl_home', 'type': 'float', 'value': 600},
            {'title': 'Slit Width (um):', 'name': 'slit_width', 'type': 'int', 'value': 100, 'min': 0},
            {'title': 'Input Port:', 'name': 'input_port', 'type': 'list', 'limits': ['direct', 'side']},
            {'title': 'Output Port:', 'name': 'output_port', 'type': 'list', 'limits': ['direct', 'side']},
            {'title': 'Grating Settings:', 'name': 'grating_settings', 'type': 'group', 'expanded': True, 'children': [
                {'title': 'Grating:', 'name': 'grating', 'type': 'list'},
                {'title': 'Lines (/mm):', 'name': 'lines', 'type': 'int', 'readonly': True},
                {'title': 'Blaze WL (nm):', 'name': 'blaze', 'type': 'str', 'readonly': True},
                {'title': 'Offset (steps):', 'name': 'grating_offset', 'type': 'int'},
            ]},
            {'title': 'Flip wavelength axis:', 'name': 'flip_wavelength', 'type': 'bool', 'value': False,
                'visible': False},
            {'title': 'Go to zero order:', 'name': 'zero_order', 'type': 'bool'},
        ]},
    ]
    params = spectro_params + comon_parameters_fun(is_multiaxes, axis_names=_axis_names, epsilon=_epsilon)

    def ini_attributes(self):

        self.shamrock_controller: ShamrockSpectrograph = None
        self.gratings_list = []

    def get_actuator_value(self) -> DataActuator:
        """Get the current value from the hardware with scaling conversion.

        Returns
        -------
        float: The position obtained after scaling conversion.
        """
        pos = DataActuator(data=self.shamrock_controller.get_wavelength(),  # when writing your own plugin replace this line
                           units='m')
        pos = self.get_position_with_scaling(pos)
        return pos

    def close(self):
        """Terminate the communication protocol"""
        if self.is_master:
            self.shamrock_controller.close()

    def commit_settings(self, param):
        try:
            if param.name() == 'spectro_sn':
                self.ini_stage()

            elif param.name() == 'grating':
                self.get_set_grating(self.grating_list.index(param.value())+1)

            elif param.name() == 'grating_offset':
                self.shamrock_controller.set_grating_offset(param.value())

            elif param.name() == 'spectro_wl':
                self.emit_status(ThreadCommand('show_splash', "Setting wavelength please wait"))
                self.shamrock_controller.set_wavelength(param.value()*1e-9)
                self.emit_status(ThreadCommand('close_splash'))

            elif param.name() == 'zero_order':
                self.emit_status(ThreadCommand('show_splash', "Moving to zero order please wait"))
                self.shamrock_controller.goto_zero_order()
                self.emit_status(ThreadCommand('close_splash'))

            elif param.name() == 'slit_width':
                self.emit_status(ThreadCommand('show_splash', "Setting slit width please wait"))
                self.shamrock_controller.set_slit_width(param.value()*1e-6)
                self.emit_status(ThreadCommand('close_splash'))

            elif param.name() == 'input_port':
                self.emit_status(ThreadCommand('show_splash', "Setting input port please wait"))
                self.shamrock_controller.set_flipper_port('input', param.value())
                self.emit_status(ThreadCommand('close_splash'))

            elif param.name() == 'output_port':
                self.emit_status(ThreadCommand('show_splash', "Setting output port please wait"))
                self.shamrock_controller.set_flipper_port('output', param.value())
                self.emit_status(ThreadCommand('close_splash'))

        except Exception as e:
            self.emit_status(ThreadCommand('Update_Status', [str(e), 'log']))

    def ini_stage(self, controller=None):
        """Actuator communication initialization

        Parameters
        ----------
        controller: (object)
            custom object of a PyMoDAQ plugin (Slave case). None if only one actuator by controller (Master case)

        Returns
        -------
        info: str
        initialized: bool
            False if initialization failed otherwise True
        """
        if self.is_master:
            idx = SPEC_NAMES.index(self.settings['spectro_sn'])
            self.shamrock_controller = ShamrockSpectrograph(idx=idx)
            initialized = self.ini_spectro()

        else:
            self.shamrock_controller = controller
            initialized = True

        info = "Spectrometer initialized"
        return info, initialized

    def move_abs(self, value: DataActuator):
        """ Move the actuator to the absolute target defined by value

        Parameters
        ----------
        value: (float) value of the absolute target positioning
        """

        value = self.check_bound(value)  #if user checked bounds, the defined bounds are applied here
        self.target_value = value
        value = self.set_position_with_scaling(value)  # apply scaling if the user specified one

        self.shamrock_controller.set_wavelength(value.value('m'))  # when writing your own plugin replace this line
        self.emit_status(ThreadCommand('Update_Status', ['Central wavelength updated']))

    def move_rel(self, value: DataActuator):
        """ Move the actuator to the relative target actuator value defined by value

        Parameters
        ----------
        value: (float) value of the relative target positioning
        """
        value = self.check_bound(self.current_position + value) - self.current_position
        self.target_value = value + self.current_position
        target_value = self.set_position_relative_with_scaling(self.target_value)

        self.controller.set_wavelength(target_value.value('m'))
        self.emit_status(ThreadCommand('Update_Status', ['Central wavelength updated']))

    def move_home(self):
        """Call the reference method of the controller"""

        home = DataActuator(data=self.settings['spectro_settings', 'spectro_wl_home'], units='nm')
        self.shamrock_controller.set_wavelength(home.value('m'))
        self.emit_status(ThreadCommand('Update_Status', ['Spectrometer at zero order']))

    def stop_motion(self):
        """Stop the actuator and emits move_done signal"""
        self.move_done() # to let the interface know the actuator stopped. Direct call as the setwavelength call is
        # blocking anyway

    def ini_spectro(self):
        # get/set grating info
        n_gratings = self.shamrock_controller.get_gratings_number()
        for i in range(n_gratings):
            info = self.shamrock_controller.get_grating_info(i)
            self.gratings_list.append(info[0])
        self.settings.child('spectro_settings', 'grating_settings', 'grating').setLimits(self.grating_list)

        idx = self.shamrock_controller.get_grating()
        self.get_set_grating(idx=idx)

        # get/set input/output port info
        if self.shamrock_controller.is_flipper_present('input'):
            input = self.shamrock_controller.get_flipper_port('input')
            self.settings.child('spectro_settings', 'input_port').setValue(input)
        else:
            self.settings.child('spectro_settings', 'input_port').hide()

        if self.shamrock_controller.is_flipper_present('output'):
            output = self.shamrock_controller.get_flipper_port('output')
            self.settings.child('spectro_settings', 'output_port').setValue(output)
        else:
            self.settings.child('spectro_settings', 'output_port').hide()

        #check if auto slitwidth is present
        if self.shamrock_controller.is_slit_present():
            width = self.shamrock_controller.get_slit_width()*1e6
            self.settings.child('spectro_settings', 'slit_width').setValue(width)
        else:
            self.settings.child('spectro_settings', 'slit_width').hide()

    def get_set_grating(self, idx):
        # idx starts at 1 (hardware specification)

        self.emit_status(ThreadCommand('show_splash', "Moving grating please wait"))
        self.shamrock_controller.set_grating(idx)
        idx = self.shamrock_controller.get_grating()

        info = self.shamrock_controller.get_grating_info(idx)
        self.settings.child('spectro_settings', 'grating_settings', 'grating').setValue(info[0])
        self.settings.child('spectro_settings', 'grating_settings', 'lines').setValue(info[0])
        self.settings.child('spectro_settings', 'grating_settings', 'blaze').setValue(info[1])
        self.settings.child('spectro_settings', 'grating_settings', 'grating_offset').setValue(info[3])

        (wl_min_m, wl_max_m) = self.shamrock_controller.get_wavelength_limits()
        wl_min = wl_min_m*1e9
        wl_max = wl_max_m*1e9
        self.settings.child('spectro_settings',
                            'spectro_wl').setOpts(limits=(wl_min, wl_max),
                                                  tip=f'Possible values are within {wl_min} and {wl_max} for'
                                                      f' the selected grating')
        self.settings.child('spectro_settings',
                            'spectro_wl_home').setOpts(limits=(wl_min, wl_max),
                                                       tip=f'Possible values are within {wl_min} and {wl_max} for'
                                                           f' the selected grating')
        self.emit_status(ThreadCommand('close_splash'))


if __name__ == '__main__':
    main(__file__)