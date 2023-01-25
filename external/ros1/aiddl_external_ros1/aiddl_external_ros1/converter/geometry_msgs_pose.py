from aiddl_core.representation import Real
from aiddl_core.representation import List
from aiddl_core.representation import KeyVal

from aiddl_external_ros1.converter.constant import POINT, X, Y, Z, ORIENTATION, W


def pose_2_aiddl(pose_msg):
    return List(
        KeyVal(POINT,
               List(KeyVal(X, Real(pose_msg.position.x)),
                    KeyVal(Y, Real(pose_msg.position.y)),
                    KeyVal(Z, Real(pose_msg.position.z)))),
        KeyVal(ORIENTATION,
               List(KeyVal(X, Real(pose_msg.orientation.x)),
                    KeyVal(Y, Real(pose_msg.orientation.y)),
                    KeyVal(Z, Real(pose_msg.orientation.z)),
                    KeyVal(W, Real(pose_msg.orientation.w)),
                    ))
    )