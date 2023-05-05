from aiddl_core.representation import Str
from aiddl_core.representation import Int
from aiddl_core.representation import List
from aiddl_core.representation import KeyVal
from aiddl_external_ros1.converter.constant import POINT, X, Y, Z, ORIENTATION, W, SEQ, FRAME_ID, STAMP

from std_msgs.msg import Header

from aiddl_external_ros1.converter.std_msgs_stamp import StampConverter


class HeaderConverter:
    @staticmethod
    def ros2aiddl(header_msg):
        return List(
            KeyVal(SEQ, Int(header_msg.seq)),
            KeyVal(STAMP, StampConverter.ros2aiddl(header_msg.stamp)),
            KeyVal(FRAME_ID, Str(header_msg.frame_id)))


    @staticmethod
    def aiddl2ros(header):
        return Header(
            seq=header[SEQ].unpack(),
            stamp=StampConverter.aiddl2ros(header[STAMP]),
            frame_id=header[FRAME_ID].unpack())
