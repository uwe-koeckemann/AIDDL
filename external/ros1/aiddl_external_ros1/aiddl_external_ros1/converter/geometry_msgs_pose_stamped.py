from aiddl_core.representation import Real
from aiddl_core.representation import Int
from aiddl_core.representation import List
from aiddl_core.representation import KeyVal

from aiddl_external_ros1.converter import PoseConverter
from aiddl_external_ros1.converter.constant import POSE, HEADER

from geometry_msgs.msg import PoseStamped

from aiddl_external_ros1.converter.std_msgs_header import HeaderConverter


class PoseStampedConverter:
    @staticmethod
    def ros2aiddl(pose_msg):
        return List(
            KeyVal(HEADER, HeaderConverter.ros2aiddl(pose_msg.header)),
            KeyVal(POSE, PoseConverter.ros2aiddl(pose_msg.pose))
        )

    @staticmethod
    def aiddl2ros(pose):
        return PoseStamped(
            HeaderConverter.aiddl2ros(pose[HEADER]),
            PoseConverter.aiddl2ros(pose[POSE]))
