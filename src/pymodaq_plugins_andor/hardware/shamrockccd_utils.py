from pylablib.devices.Andor.AndorSDK2 import AndorSDK2Camera
from pylablib.devices.Andor.Shamrock import ShamrockSpectrograph

class ShamrockCCDCompo(AndorSDK2Camera):
    def __init__(self, cam_idx=0, spec_idx=0):
        super().__init__(idx=cam_idx)
        self.shamrock = ShamrockSpectrograph(idx=spec_idx)