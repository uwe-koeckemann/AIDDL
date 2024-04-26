import grpc
import aiddl_external_grpc_python.generated.empty_pb2 as empty_pb2

import aiddl_external_grpc_python.generated.sensor_pb2_grpc as sensor_pb2_grpc
from aiddl_external_grpc_python.converter import Converter


class SensorClient:
    def __init__(self, host, port, container):
        channel = grpc.insecure_channel(f'{host}:{port}')
        self.stub = sensor_pb2_grpc.SensorStub(channel)
        self.converter = Converter(container)

    def sense(self):
        answer = self.stub.Sense(empty_pb2.Empty())
        return (self.converter.pb2aiddl(answer.value), answer.sequence_id, answer.timestamp_nano)

    def get_latest_sensor_value(self):
        answer = self.stub.GetLatestSensorValue(empty_pb2.Empty())
        return (self.converter.pb2aiddl(answer.value), answer.sequence_id, answer.timestamp_nano)

