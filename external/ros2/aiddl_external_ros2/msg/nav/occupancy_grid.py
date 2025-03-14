from aiddl_core.representation import Int
from aiddl_core.representation import List
from aiddl_core.representation import KeyVal

from aiddl_external_ros2.msg.const import INFO, DATA
from aiddl_external_ros2.msg.nav.map_meta_data import MapMetaDataConverter
from nav_msgs.msg import OccupancyGrid


class OccupancyGridConverter:
    @staticmethod
    def ros2aiddl(msg: OccupancyGrid):
        return List(
            KeyVal(INFO, MapMetaDataConverter.ros2aiddl(msg.map.info)),
            KeyVal(DATA, List([Int(x) for x in msg.map.data]))
        )
