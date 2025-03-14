from aiddl_core.representation import Real
from std_msgs.msg import Float64


class Float64Converter:
    @staticmethod
    def ros2aiddl(msg):
        return Real(msg.data)

    @staticmethod
    def aiddl2ros(real_term: Real):
        msg = Float64()
        msg.data = real_term.unpack()
        return msg
