from nav_msgs.srv import GetMap

from aiddl_external_ros2.msg.nav.occupancy_grid import OccupancyGridConverter

class GetMapConverter(object):
    @staticmethod
    def request_aiddl2ros(term):
        return GetMap.Request()

    @staticmethod
    def result_ros2aiddl(msg):
        return OccupancyGridConverter.ros2aiddl(msg)
