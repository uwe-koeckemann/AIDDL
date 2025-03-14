from aiddl_core.representation import Real
from aiddl_core.representation import List
from aiddl_core.representation import KeyVal
from aiddl_external_ros2.msg.geometry.const import POSE, POINT, X, Y, Z, ORIENTATION, W

from geometry_msgs.msg import Pose, Point, Quaternion

class PoseConverter:
    @staticmethod
    def ros2aiddl(pose_msg):
        return List(
            KeyVal(POINT,
                   List(KeyVal(X, Real(pose_msg.position.x)),
                        KeyVal(Y, Real(pose_msg.position.y)),
                        KeyVal(Z, Real(pose_msg.position.z)))),
            KeyVal(ORIENTATION,
                   List(KeyVal(X, Real(pose_msg.orientation.x)),
                        KeyVal(Y, Real(pose_msg.orientation.y)),
                        KeyVal(Z, Real(pose_msg.orientation.z)),
                        KeyVal(W, Real(pose_msg.orientation.w)))))

    @staticmethod
    def aiddl2ros(pose):
        msg = Pose()
        msg.position = Point()
        msg.orientation = Quaternion()
        msg.position.x = pose[POINT][X].unpack()
        msg.position.y = pose[POINT][Y].unpack()
        msg.position.z = pose[POINT][Z].unpack()
        msg.orientation.x = pose[ORIENTATION][X].unpack()
        msg.orientation.y = pose[ORIENTATION][Y].unpack()
        msg.orientation.z = pose[ORIENTATION][Z].unpack()
        msg.orientation.w = pose[ORIENTATION][W].unpack()

        return msg
