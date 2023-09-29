from enum import Enum
import grpc

import aiddl_external_grpc_python.generated.receiver_pb2 as receiver_pb2
from aiddl_external_grpc_python.converter import Converter


class Order(Enum):
    OLDEST_FIRST = 1
    NEWEST_FIRST = 2


class ReceiverClient:
    def __init__(self, host, port, container,
                 pull_order=Order.OLDEST_FIRST,
                 sort_by=Order.OLDEST_FIRST,
                 pull_max=1,
                 flush_queue=False):
        self.query = None
        channel = grpc.insecure_channel(f'{host}:{port}')
        self.stub = receiver_pb2.SenderStub(channel)
        self.converter = Converter(container)
        self.configureQuery(sort_by=sort_by,
                            pull_order=pull_order,
                            pull_max=pull_max,
                            flush_queue=flush_queue)

    def configure_query(self,
                        sort_by=Order.OLDEST_FIRST,
                        pull_order=Order.OLDEST_FIRST,
                        pull_max=1,
                        flush_queue=False):
        self.query = receiver_pb2.Query()
        if sort_by == Order.OLDEST_FIRST:
            self.query.sort_by = receiver_pb2.OLDEST_FIRST
        else:
            self.query.sort_by = receiver_pb2.NEWEST_FIRST
        if pull_order == Order.OLDEST_FIRST:
            self.query.pull_order = receiver_pb2.OLDEST_FIRST
        else:
            self.query.pull_order = receiver_pb2.NEWEST_FIRST
        self.query.pull_max = pull_max
        self.query.flush_queue = flush_queue

    def Receive(self):
        answer = self.stub.receive(self.query)
        return [self.converter.pb2aiddl(m) for m in answer.messages]
