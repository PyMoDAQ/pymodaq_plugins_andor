# Legacy plugins

Former instrument plugins of this package, kept for reference: the Andor SDK2 camera (`daq_2Dviewer_AndorCCD`), the Shamrock spectrometer (`daq_move_Shamrock`) and the Shamrock CCD detectors (`daq_1Dviewer_ShamrockCCD`, `daq_1Dviewer_ShamrockCCDLegacy`).

They were replaced by the pylablib based plugins (`daq_2Dviewer_AndorCCDPll`, `daq_move_ShamrockPll`, `daq_1Dviewer_ShamrockCCDPll`).

This folder is outside `daq_move_plugins` and `daq_viewer_plugins`, so PyMoDAQ does not load these modules: they are not runnable as they are. To use one again, move it back to the matching plugin folder and fix its imports (the imports between the modules of this folder point here).
