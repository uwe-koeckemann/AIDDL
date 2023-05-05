from aiddl_core.representation import Real
from aiddl_core.representation import Int
from aiddl_core.representation import Tuple
from aiddl_core.representation import List
from aiddl_core.representation import KeyVal
from aiddl_external_ros1.converter.constant import SEC, NSEC

from rospy import Time


class StampConverter:
    @staticmethod
    def ros2aiddl(stamp_msg):
        return List(
            KeyVal(SEC, Int(stamp_msg.sec)),
            KeyVal(NSEC, Int(stamp_msg.nsec)))

    @staticmethod
    def aiddl2ros(stamp):
        return Time(
            secs=stamp[SEC].unpack(),
            nsecs=stamp[NSEC].unpack())
