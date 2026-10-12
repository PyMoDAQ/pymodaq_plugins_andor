
from pymodaq_utils.config import GlobalConfig
from pymodaq_utils.logger import get_module_name, set_logger

from pylablib.devices.Andor import AndorSDK3Camera
from pylablib.devices import Andor

import pylablib as pll


logger = set_logger(get_module_name(__file__))
config = GlobalConfig()


if config('andor', 'sdk3', 'dll_path') != "":
    pll.par["devices/dlls/andor_sdk3"] = config('andor', 'sdk3', 'dll_path')


def get_camera_names():
    camera_list = []
    try:
        n_camera = Andor.get_cameras_number_SDK3()
        for ind_cam in range(n_camera):
            try:
                cam = AndorSDK3Camera(idx=ind_cam)
                model = cam.get_attribute_value('CameraModel')
                name = cam.get_attribute_value('CameraName')
                serial_number = cam.get_attribute_value('SerialNumber')
                camera_list.append(f'{model} {serial_number}')
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

