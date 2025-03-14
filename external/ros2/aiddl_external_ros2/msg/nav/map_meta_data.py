from aiddl_core.representation import Real
from aiddl_core.representation import List
from aiddl_core.representation import KeyVal

from aiddl_external_ros2.msg.nav.const import RESOLUTION, WIDTH, HEIGHT, ORIGIN
from aiddl_external_ros2.msg.geometry.pose import PoseConverter

from nav_msgs.msg import MapMetaData


class MapMetaDataConverter(object):
    @staticmethod
    def ros2aiddl(msg: MapMetaData):
        return List(
            KeyVal(RESOLUTION, Real(msg.resolution)),
            KeyVal(WIDTH, Real(msg.width)),
            KeyVal(HEIGHT, Real(msg.height)),
            KeyVal(ORIGIN, PoseConverter.ros2aiddl(msg.origin))
        )
