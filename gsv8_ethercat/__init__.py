"""
GSV8 EtherCAT Package

Python interface for ME-Messsysteme GSV-8 force/torque sensor amplifier via EtherCAT
"""

from .sensor import GSV8Sensor, GSV8Data
from .calibration import GSV8Calibration
from .monitor import GSV8Monitor
from .diagnostics import GSV8Diagnostics, ChannelStatistics

__version__ = "0.1.0"
__author__ = "Konstantinos Katsampiris Salgado"
__license__ = "MIT"

__all__ = [
    "GSV8Sensor",
    "GSV8Data",
    "GSV8Calibration",
    "GSV8Monitor",
    "GSV8Diagnostics",
    "ChannelStatistics",
]
