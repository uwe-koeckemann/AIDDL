from action_tutorials_interfaces.action import Fibonacci

from aiddl_core.representation import Int, List


class FibonacciConverter(object):
    @staticmethod
    def request_aiddl2ros(term):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = int(term.unpack())
        return goal_msg

    @staticmethod
    def feedback_ros2aiddl(msg):
        seq = [Int(x) for x in msg.feedback.partial_sequence]
        return List(seq)

    @staticmethod
    def result_ros2aiddl(msg):
        print(msg)
        seq = [Int(x) for x in msg.result.sequence]
        return List(seq)
