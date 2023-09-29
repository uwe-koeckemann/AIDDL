from aiddl_core.representation import Int
from aiddl_core.representation import List
from aiddl_core.representation import KeyVal

from aiddl_external_ros1.converter.constant import INFO, DATA
from aiddl_external_ros1.converter.nav_msgs_map_meta_data import map_meta_data_2_aiddl

def occupancy_grid_2_aiddl(map_msg):
    return List(
        KeyVal(INFO, map_meta_data_2_aiddl(map_msg.info)),
        KeyVal(DATA, List([Int(x) for x in map_msg.data]))
    )
