import grpc

import aiddl_external_grpc_python.generated.actor_pb2 as actor_pb2
import aiddl_external_grpc_python.generated.actor_pb2_grpc as actor_pb2_grpc
from aiddl_external_grpc_python.converter import Converter
from aiddl_external_grpc_python.generated.actor_pb2 import REJECTED


class ActorClient:
    def __init__(self, host, port, container):
        channel = grpc.insecure_channel(f'{host}:{port}')
        self.stub = actor_pb2_grpc.ActorStub(channel)
        self.converter = Converter(container)

    def is_supported(self, action):
        answer = self.stub.IsSupported(self.converter.aiddl2pb(action))
        return answer.is_supported

    def dispatch(self, action):
        status = self.stub.Dispatch(self.converter.aiddl2pb(action))
        if status == REJECTED:
            return None
        else:
            return status.id

    def get_status(self, action_id):
        request = actor_pb2.Id()
        request.id = action_id
        status = self.stub.GetStatus(request)
        return status.state, self.converter.pb2aiddl(status.feedback), status.msg

    def cancel(self, action_id):
        request = actor_pb2.Id()
        request.id = action_id
        status = self.stub.Cancel(request)
        return status.state, self.converter.pb2aiddl(status.feedback), status.msg


