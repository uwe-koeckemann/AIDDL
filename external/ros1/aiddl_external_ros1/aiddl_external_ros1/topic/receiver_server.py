from concurrent import futures

import grpc
import rospy

from aiddl_core.parser import parser
import aiddl_external_ros1.proto.aiddl_msg_pb2 as aiddl_msg_pb2
import aiddl_external_ros1.proto.receiver_pb2 as receiver_pb2
import aiddl_external_ros1.proto.receiver_pb2_grpc as receiver_pb2_grpc

from aiddl_external_grpc_python.receiver import ReceiverServer

class TopicReceiverServer(ReceiverServer):
    def __init__(self, port,
                 ros_topic,
                 ros_msg_type,
                 f_convert,
                 verbose=False):
        super.__init__(port)
        self.f_convert = f_convert
        self.verbose = verbose
        rospy.Subscriber(ros_topic, ros_msg_type, self._callback)
        self.queue_lock = False
        self.message_queue = []

    def _callback(self, data):
        while self.queue_lock:
            pass
        self.queue_lock = True
        self.message_queue.append(data)
        self.queue_lock = False

    def receive(self, request, context):
        sort_by = request.sortBy
        pull_order = request.pullOrder
        receive_max = request.receiveMax
        flush_queue = request.flushQueue

        while self.queue_lock:
            pass
        self.queue_lock = True

        n_receive = receive_max
        if n_receive == -1:
            n_receive = len(self.message_queue)

        pulled = []

        while len(pulled) < n_receive and len(self.message_queue) > 0:
            if pull_order == receiver_pb2.OldestFirst:
                item = self.message_queue[0]
                del self.message_queue[0]
            elif pull_order == receiver_pb2.NewestFirst:
                item = self.message_queue[-1]
                del self.message_queue[-1]
            pulled.append(item)
        if flush_queue:
            self.message_queue = []

        self.queue_lock = False

        if sort_by == receiver_pb2.OldestFirst and pull_order == receiver_pb2.NewestFirst:
            pulled.reverse()
        elif sort_by == receiver_pb2.NewestFirst and pull_order == receiver_pb2.OldestFirst:
            pulled.reverse()

        answer = [self.f_convert(x) for x in pulled]
        return receiver_pb2.Messages(messages=answer)
