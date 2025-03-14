from aiddl_core.representation import Str
from aiddl_core.representation import List
from aiddl_core.representation import KeyVal
from aiddl_external_ros2.msg.const import FRAME_ID, STAMP

from std_msgs.msg import Header

from aiddl_external_ros2.msg.builtin.time import TimeConverter


class HeaderConverter:
    @staticmethod
    def ros2aiddl(header_msg):
        return List(
            #KeyVal(SEQ, Int(header_msg.seq)),
            KeyVal(STAMP, TimeConverter.ros2aiddl(header_msg.stamp)),
            KeyVal(FRAME_ID, Str(header_msg.frame_id)))

    @staticmethod
    def aiddl2ros(header):
        msg = Header()
        #msg.seq = header[SEQ].unpack()
        msg.stamp = TimeConverter.aiddl2ros(header[STAMP])
        msg.frame_id = header[FRAME_ID].unpack()
        return msg
