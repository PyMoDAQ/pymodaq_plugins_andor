pymodaq_plugins_andor (Andor)
#############################

.. image:: https://img.shields.io/pypi/v/pymodaq_plugins_andor.svg
   :target: https://pypi.org/project/pymodaq_plugins_andor/
   :alt: Latest Version

.. image:: https://readthedocs.org/projects/pymodaq/badge/?version=latest
   :target: https://pymodaq.readthedocs.io/en/stable/?badge=latest
   :alt: Documentation Status

.. image:: https://github.com/PyMoDAQ/pymodaq_plugins_andor/workflows/Upload%20Python%20Package/badge.svg
    :target: https://github.com/PyMoDAQ/pymodaq_plugins_andor

Set of PyMoDAQ plugins for Andor Camera (CCD camera using SDK2, SCMOS cameras using SDK3...)


Authors
=======

* Sébastien. J. Weber

Instruments
===========
Below is the list of instruments included in this plugin

Actuators
+++++++++

* **Shamrock**: Shamrock series of spectrometer used as a monochromator

Viewer1D
++++++++

* **ShamrockCCD**: Shamrock series of spectrometer using the Andor CCD cameras (built using double inheritance)
* **ShamrockSCMOS**: Shamrock series of spectrometer using the Andor SCMOS cameras (Not tested) (built using double inheritance)
* **ShamrockCCDComposition**: Shamrock series of spectrometer using the Andor CCD cameras (built using CCD camera inheritance and Shamrock composition)
* **ShamrockSCMOSComposition**: Shamrock series of spectrometer using the Andor SCMOS cameras (Not tested) (built using SCMOS camera inheritance and Shamrock composition)

Viewer2D
++++++++

* **AndorCCD**: Andor CCD camera using the SDK2
* **AndorSCMOS**: Andor CCD camera using the SDK3

