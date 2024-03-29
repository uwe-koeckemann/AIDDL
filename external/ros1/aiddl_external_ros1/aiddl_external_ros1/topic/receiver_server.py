import logging
import os
import atexit
import rospy

import aiddl_external_grpc_python.generated.receiver_pb2 as receiver_pb2
from aiddl_core.container import Container
from aiddl_external_grpc_python.converter import Converter
from aiddl_external_grpc_python.receiver import ReceiverServer


def run_topic_receiver(node_name, ros_msg_type, ros_2_aiddl, verbose=False):
    grpc_port = int(os.getenv("GRPC_PORT"))
    topic = os.getenv("ROS_TOPIC")
    logging.info(f'Starting {node_name} as receiver for {ros_msg_type} from topic "{topic}" to AIDDL gRPC receiver port {grpc_port}')
    
    rospy.init_node(node_name, anonymous=True)
    server = TopicReceiverServer(
        grpc_port,
        topic,
        ros_msg_type,
        ros_2_aiddl)

    def exit_handler():
        print('Closing down...')
        server.server.stop(2).wait()
        print('Done.')
    atexit.register(exit_handler)

    print('Starting server...')
    server.start()
    print('Running.')
    rospy.spin()


class TopicReceiverServer(ReceiverServer):
    def __init__(self, port,
                 ros_topic,
                 ros_msg_type,
                 f_convert,
                 verbose=False):
        super(TopicReceiverServer, self).__init__(port)
        self.f_convert = f_convert
        self.verbose = verbose
        rospy.Subscriber(ros_topic, ros_msg_type, self._callback)
        self.queue_lock = False
        self.message_queue = []
        container = Container()
        self.converter = Converter(container)

    def _callback(self, data):
        while self.queue_lock:
            pass
        self.queue_lock = True
        self.message_queue.append(data)
        self.queue_lock = False

    def Receive(self, request, context):
        sort_by = request.sort_by
        pull_order = request.pull_order
        receive_max = request.pull_max
        flush_queue = request.flush_queue

        while self.queue_lock:
            pass
        self.queue_lock = True

        n_receive = receive_max
        if n_receive == -1:
            n_receive = len(self.message_queue)

        pulled = []

        while len(pulled) < n_receive and len(self.message_queue) > 0:
            if pull_order == receiver_pb2.OLDEST_FIRST:
                item = self.message_queue[0]
                del self.message_queue[0]
            elif pull_order == receiver_pb2.NEWEST_FIRST:
                item = self.message_queue[-1]
                del self.message_queue[-1]
            pulled.append(item)
        if flush_queue:
            self.message_queue = []

        self.queue_lock = False

        if sort_by == receiver_pb2.OLDEST_FIRST and pull_order == receiver_pb2.NEWEST_FIRST:
            pulled.reverse()
        elif sort_by == receiver_pb2.NEWEST_FIRST and pull_order == receiver_pb2.OLDEST_FIRST:
            pulled.reverse()

        answer = [self.converter.aiddl2pb(self.f_convert(x)) for x in pulled]
        return receiver_pb2.Messages(messages=answer)
