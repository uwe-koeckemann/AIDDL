from aiddl_core.representation import Int
from aiddl_core.representation import List
from aiddl_core.representation import KeyVal
from aiddl_external_ros2.msg.builtin.const import SECS, NSECS

from builtin_interfaces.msg import Time

import rclpy

class TimeConverter:
    @staticmethod
    def ros2aiddl(msg: Time):
        return List(
            KeyVal(SECS, Int(msg.sec)),
            KeyVal(NSECS, Int(msg.nanosec)))

    @staticmethod
    def aiddl2ros(stamp) -> Time:
        msg = Time()
        msg.sec = stamp[SECS].unpack()
        msg.nanosec = stamp[NSECS].unpack()
        return msg
