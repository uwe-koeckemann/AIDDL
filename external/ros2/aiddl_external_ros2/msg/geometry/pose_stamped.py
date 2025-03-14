from aiddl_core.representation import List
from aiddl_core.representation import KeyVal

from aiddl_external_ros2.msg.geometry import PoseConverter
from aiddl_external_ros2.msg.geometry.const import POSE
from aiddl_external_ros2.msg.std.const import HEADER

from geometry_msgs.msg import PoseStamped

from aiddl_external_ros2.msg.std.header import HeaderConverter


class PoseStampedConverter:
    @staticmethod
    def ros2aiddl(pose_msg):
        return List(
            KeyVal(HEADER, HeaderConverter.ros2aiddl(pose_msg.header)),
            KeyVal(POSE, PoseConverter.ros2aiddl(pose_msg.pose))
        )

    @staticmethod
    def aiddl2ros(pose):
        print(pose)
        msg = PoseStamped()
        msg.header = HeaderConverter.aiddl2ros(pose[HEADER])
        msg.pose = PoseConverter.aiddl2ros(pose[POSE])
        return msg
