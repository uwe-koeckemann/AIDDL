from aiddl_core.representation import Str

from std_msgs.msg import String

class StringConverter:
    @staticmethod
    def ros2aiddl(msg: String):
        return Str(msg)

    @staticmethod
    def aiddl2ros(string) -> String:
        return string.string
