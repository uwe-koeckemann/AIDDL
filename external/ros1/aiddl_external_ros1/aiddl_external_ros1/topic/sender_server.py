import os
import rospy
import atexit
import logging

from aiddl_external_grpc_python.sender import SenderServer
import aiddl_external_grpc_python.generated.empty_pb2 as empty_pb2

def run_topic_sender(node_name, ros_msg_type, aiddl_2_ros, verbose=False):
    grpcport = int(os.getenv("GRPC_PORT"))
    topic = os.getenv("ROS_TOPIC")
    logging.info(f'Starting {node_name} as sender for {ros_msg_type} to topic "{topic}" from AIDDL gRPC receiver port {grpcport}')
    pub = rospy.Publisher(topic, ros_msg_type, queue_size=10)
        
    rospy.init_node(node_name, anonymous=True)
    logging.info('Creating sender server...')
    server = TopicSenderServer(grpcport,
                               pub,
                               aiddl_2_ros,
                               verbose=verbose)
    def exit_handler():
        logging.info('Closing down...')
        server.server.stop(2).wait()
        logging.info('Done.')
    atexit.register(exit_handler)
    logging.info('Starting server...')
    server.start()
    logging.info('Running.')
    rospy.spin()


class TopicSenderServer(SenderServer):
    def __init__(self, port, publisher, f_convert, verbose=False):
        super(TopicSenderServer, self).__init__(port)
        self.pub = publisher
        self.f_convert = f_convert
        self.verbose = verbose
      
    def Send(self, request, context):
        if self.verbose:
            logging.info("Sending:", request)
        ros_msg = self.f_convert(request)
        if self.verbose:
            logging.info("Converted to:", ros_msg)
        self.pub.publish(ros_msg)
        return empty_pb2.Empty()
