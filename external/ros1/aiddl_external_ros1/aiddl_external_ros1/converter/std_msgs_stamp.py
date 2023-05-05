from aiddl_core.representation import Real
from aiddl_core.representation import Int
from aiddl_core.representation import Tuple
from aiddl_core.representation import List
from aiddl_core.representation import KeyVal
from aiddl_external_ros1.converter.constant import POINT, X, Y, Z, ORIENTATION, W, SEQ, FRAME_ID, SECS, NSECS

from rospy import Time


class StampConverter:
    @staticmethod
    def ros2aiddl(stamp_msg):
        return List(
            KeyVal(SECS, Int(stamp_msg.secs)),
            KeyVal(NSECS, Int(stamp_msg.nsecs)))


    @staticmethod
    def aiddl2ros(pose):
        return Time(
            secs=pose[SECS].unpack(),
            nsecs=pose[NSECS].unpack())
