from aiddl_core.representation import Str
from aiddl_core.representation import Sym

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import String

from aiddl_external_ros2.msg.geometry import PointConverter
from aiddl_external_ros2.msg.geometry import PoseConverter

from aiddl_external_ros2.msg.geometry.const import POSE, POINT, X, Y, Z, ORIENTATION, W
from aiddl_external_ros2.msg.visualization.const import SCALE, ID, TYPE, ACTION, TEXT, COLOR, R, B, G, A

type_look_up = {Sym('arrow'): Marker.ARROW, Sym('cube'): Marker.CUBE, Sym('sphere'): Marker.SPHERE,
                Sym('cylinder'): Marker.CYLINDER, Sym('line-strip'): Marker.LINE_STRIP,
                Sym('line-list'): Marker.LINE_LIST, Sym('cube-list'): Marker.CUBE_LIST,
                Sym('sphere-list'): Marker.SPHERE_LIST, Sym('points'): Marker.POINTS,
                Sym('text-view-facing'): Marker.TEXT_VIEW_FACING, Sym('mesh'): Marker.MESH_RESOURCE,
                Sym('triangle-list'): Marker.TRIANGLE_LIST}

action_look_up = {Sym('add'): Marker.ADD, Sym('modify'): Marker.MODIFY, Sym('delete'): Marker.DELETE,
                  Sym('delete-all'): Marker.DELETEALL}

class MarkerConverter:
    @staticmethod
    def aiddl2ros(g):
        marker = Marker()
        marker.header.frame_id = g[FRAME_ID].unpack()
        marker.ns = g[NAMESPACE].unpack()
        marker.id = g[ID].unpack()
        marker.type = type_look_up[g[TYPE]]
        marker.action = action_look_up[g[ACTION]]
        marker.pose = PoseConverter.aiddl2ros(g[POSE])

        if g[POINTS] is not None:
            marker.points = [PointConverter.aiddl2ros(p) for p in g[POINTS]]

        if g[TEXT] is not None:
            text = g[TEXT]
            if isinstance(text, Str):
                text = text.string
            else:
                text = str(text)
            marker.text = text

        marker.scale = Vector3()
        marker.scale.x = g[SCALE][X].unpack()
        marker.scale.y = g[SCALE][Y].unpack()
        marker.scale.z = g[SCALE][Z].unpack()

        marker.color = ColorRGBA()
        marker.color.r = g[COLOR][R].unpack()
        marker.color.g = g[COLOR][G].unpack()
        marker.color.b = g[COLOR][B].unpack()
        marker.color.a = g[COLOR][A].unpack()

        return marker
