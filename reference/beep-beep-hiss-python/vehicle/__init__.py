"""
车辆模型包
包含半挂车动力学模型和相关参数
"""

from .truckTrailerOnAxle import TruckTrailerOnAxle
from .get_default_truck_trailer_params import get_default_truck_trailer_params
from .sys_truck_trailer import sys_truck_trailer, sys_truck_trailer_wrapper

__all__ = [
    'TruckTrailerOnAxle',
    'get_default_truck_trailer_params',
    'sys_truck_trailer',
    'sys_truck_trailer_wrapper'
]
