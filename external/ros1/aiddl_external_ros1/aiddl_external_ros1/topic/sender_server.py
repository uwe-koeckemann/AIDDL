import os
import rospy
import atexit

from aiddl_core.parser import parser
from aiddl_external_grpc_python.sender import SenderServer
import aiddl_external_grpc_python.generated.empty_pb2 as empty_pb2


def run_topic_sender(node_name, ros_msg_type, aiddl_2_ros, verbose=False):
    grpcport = int(os.getenv("GRPC_PORT"))
    topic = os.getenv("ROS_TOPIC")
    print(f'Starting sender for {ros_msg_type} to topic "{ros_topic}" from AIDDL gRPC receiver port {grpcport}')
    pub = rospy.Publisher(topic, ros_msg_type, queue_size=10)
        
    rospy.init_node(node_name, anonymous=True)
    print('Creating sender server')
    server = TopicSenderServer(grpcport,
                               pub,
                               aiddl_2_ros,
                               verbose=verbose)
    def exit_handler():
        print('Closing down...')
        server.server.stop(2).wait()
        print('Done.')
    atexit.register(exit_handler)
    print('Starting server...')
    server.start()
    print('Running.')
    rospy.spin() # server.wait_for_termination()


class TopicSenderServer(SenderServer):
    def __init__(self, port, publisher, f_convert, verbose=False):
        super(TopicSenderServer, self).__init__(port)
        self.pub = publisher
        self.f_convert = f_convert
        self.verbose = verbose
      
    def Send(self, request, context):
        if self.verbose:
            print("Sending:", request)
        ros_msg = self.f_convert(request)
        if self.verbose:
            print("Converted to:", ros_msg)
        self.pub.publish(ros_msg)
        return empty_pb2.Empty()
