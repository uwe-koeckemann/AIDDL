from nav2_msgs.action import NavigateToPose

from aiddl_core.representation import Int, List, Term, Real, KeyVal, Sym
from aiddl_external_ros2.msg.geometry import PoseStampedConverter
from aiddl_external_ros2.msg.builtin.duration import DurationConverter


class NavigateToPoseConverter(object):
    @staticmethod
    def request_aiddl2ros(term: Term):
        msg = NavigateToPose.Goal()
        msg.pose = PoseStampedConverter.aiddl2ros(term[0])
        msg.behavior_tree = term[1].unpack()
        return msg

    @staticmethod
    def feedback_ros2aiddl(msg):
        msg = NavigateToPose.Feedback()
        current_pose_term = PoseStampedConverter.ros2aiddl(msg.current_pose)
        navigation_time = DurationConverter.ros2aiddl(msg.navigation_time)
        estimated_time_remaining = DurationConverter.ros2aiddl(msg.estimated_time_remaining)
        number_of_recoveries = Int(msg.number_of_recoveries)
        distance_remaining = Real(msg.distance_remaining)
        return List(
            KeyVal(Sym("current_pose"), current_pose_term),
            KeyVal(Sym("navigation_time"), navigation_time),
            KeyVal(Sym("estimated_time_remaining"), estimated_time_remaining),
            KeyVal(Sym("number_of_recoveries"), number_of_recoveries),
            KeyVal(Sym("distance_remaining"), distance_remaining)
        )

    @staticmethod
    def result_ros2aiddl(msg):
        return Sym("NIL")
