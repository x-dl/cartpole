# pendulum_control/__init__.py

# 从子模块中导入核心类，使其在顶层包名下可用
from .hardware.actuator import Actuator
from .hardware.encoder import EncoderReader
from .controllers import AnglePIDController, PositionPIDController
from .pendulum import InvertedPendulum

# （可选）定义 __all__ 来指定 'from pendulum_control import *' 时会导入什么
__all__ = [
    'Actuator',
    'EncoderReader',
    'AnglePIDController',
    'PositionPIDController',
    'InvertedPendulum'
]