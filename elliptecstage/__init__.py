"""
``elliptecstage``
Python module to control Elliptec Linear Translation 
Stage with Resonant Piezoelectric Motors
===================

"""
import serial
from enum import Enum
import src.elliptecstage

version_info = 0, 1, 0
__version__ = '.'.join(map(str, version_info))
