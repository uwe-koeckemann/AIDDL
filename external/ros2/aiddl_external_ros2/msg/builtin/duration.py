from aiddl_core.representation import Int, Term, List, KeyVal

from aiddl_external_ros2.msg.builtin.const import SECS, NSECS

from builtin_interfaces.msg import Duration


class DurationConverter:
    @staticmethod
    def ros2aiddl(msg: Duration):
        return List(
            KeyVal(SECS, Int(msg.sec)),
            KeyVal(NSECS, Int(msg.nanosec)))

    @staticmethod
    def aiddl2ros(duration_term: Term) -> Duration:
        msg = Duration()
        msg.sec = duration_term[SECS].unpack()
        msg.nanosec = duration_term[NSECS].unpack()
        return msg
