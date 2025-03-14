from aiddl_core.representation import Int

class Int64Converter:
    @staticmethod
    def ros2aiddl(msg):
        return Int(msg.data)

    @staticmethod
    def aiddl2ros(term):
        return int(term.unpack())