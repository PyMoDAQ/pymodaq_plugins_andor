from pymodaq_utils.config import GlobalConfig
from pymodaq_utils.logger import get_module_name, set_logger

from pylablib.devices.Andor import Shamrock

import pylablib as pll


logger = set_logger(get_module_name(__file__))
config = GlobalConfig()


#if config('andor', 'shamrock', 'dll_path') is not "":
#    pll.par["devices/dlls/andor_shamrock"] = config('andor', 'shamrock', 'dll_path')


def get_spectrometers():
    spectro_list = []
    try:
        spectro_list = Shamrock.list_spectrographs()
    except Exception as e:
        logger.exception(f'Impossible to communicate with spectrograph, try to set another library path in the preferences')

    return spectro_list