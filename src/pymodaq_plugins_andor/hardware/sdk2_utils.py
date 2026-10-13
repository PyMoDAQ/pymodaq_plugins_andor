from pymodaq_utils.config import GlobalConfig
from pymodaq_utils.logger import get_module_name, set_logger

from pylablib.devices.Andor import AndorSDK2Camera
from pylablib.devices import Andor

import pylablib as pll


logger = set_logger(get_module_name(__file__))
config = GlobalConfig()


#if config('andor', 'sdk2', 'dll_path') is not "":
#    pll.par["devices/dlls/andor_sdk2"] = config('andor', 'sdk2', 'dll_path')


def get_camera_names():
    camera_list = []
    try:
        n_camera = Andor.get_cameras_number_SDK2()
        for ind_cam in range(n_camera):
            try:
                cam = AndorSDK2Camera(idx=ind_cam)
                controller_model, head_model, serial_number = cam.get_device_info()
                camera_list.append(f'{controller_model} {head_model} {serial_number}')
            except Exception as e:
                pass
            finally:
                try:
                    cam.close()
                except:
                    pass
    except Exception as e:
        logger.exception(f'Impossible to communicate with camera, try to set another library path in the preferences')

    return camera_list
