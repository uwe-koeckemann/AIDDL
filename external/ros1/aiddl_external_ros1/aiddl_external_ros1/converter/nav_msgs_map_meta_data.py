from aiddl_core.representation import Real
from aiddl_core.representation import List
from aiddl_core.representation import KeyVal

from aiddl_external_ros1.converter.constant import RESOLUTION, WIDTH, HEIGHT, ORIGIN
from aiddl_external_ros1.converter.geometry_msgs_pose import PoseConverter

def map_meta_data_2_aiddl(map_msg):
    return List(
        KeyVal(RESOLUTION, Real(map_msg.resolution)),
        KeyVal(WIDTH, Real(map_msg.width)),
        KeyVal(HEIGHT, Real(map_msg.height)),
        KeyVal(ORIGIN, PoseConverter.pose_2_aiddl(map_msg.origin))
    )
