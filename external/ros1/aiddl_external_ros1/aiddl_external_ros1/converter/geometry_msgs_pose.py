from aiddl_core.representation import Real
from aiddl_core.representation import List
from aiddl_core.representation import KeyVal
from aiddl_external_ros1.converter.constant import POINT, X, Y, Z, ORIENTATION, W

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
        return Pose(
            Point(pose[POINT][X].unpack(),
                  pose[POINT][Y].unpack(),
                  pose[POINT][Z].unpack()),
            Quaternion(
                pose[ORIENTATION][X].unpack(),
                pose[ORIENTATION][Y].unpack(),
                pose[ORIENTATION][Z].unpack(),
                pose[ORIENTATION][W].unpack()))
