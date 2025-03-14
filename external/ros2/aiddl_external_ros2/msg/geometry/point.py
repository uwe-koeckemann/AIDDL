from aiddl_core.representation import Real
from aiddl_core.representation import List
from aiddl_core.representation import KeyVal
from aiddl_external_ros2.msg.geometry.const import POSE, POINT, X, Y, Z, ORIENTATION, W

from geometry_msgs.msg import Pose, Point, Quaternion


class PointConverter:
    @staticmethod
    def ros2aiddl(point_msg):
        return List(
            KeyVal(X, Real(point_msg.x)),
            KeyVal(Y, Real(point_msg.y)),
            KeyVal(Z, Real(point_msg.z)))

    @staticmethod
    def aiddl2ros(point):
        msg = Point()
        msg.x = point[X].unpack()
        msg.y = point[Y].unpack()
        msg.z = point[Z].unpack()

        return msg
