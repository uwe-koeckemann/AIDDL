# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: receiver.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import builder as _builder
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from . import aiddl_pb2 as aiddl__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0ereceiver.proto\x12\x17org.aiddl.external.grpc\x1a\x0b\x61iddl.proto\"\x93\x01\n\x05Query\x12/\n\x07sort_by\x18\x01 \x01(\x0e\x32\x1e.org.aiddl.external.grpc.Order\x12\x32\n\npull_order\x18\x02 \x01(\x0e\x32\x1e.org.aiddl.external.grpc.Order\x12\x10\n\x08pull_max\x18\x03 \x01(\x05\x12\x13\n\x0b\x66lush_queue\x18\x04 \x01(\x08\";\n\x08Messages\x12/\n\x08messages\x18\x01 \x03(\x0b\x32\x1d.org.aiddl.external.grpc.Term*+\n\x05Order\x12\x10\n\x0cOLDEST_FIRST\x10\x00\x12\x10\n\x0cNEWEST_FIRST\x10\x01\x32X\n\x08Receiver\x12L\n\x07Receive\x12\x1e.org.aiddl.external.grpc.Query\x1a!.org.aiddl.external.grpc.MessagesB\x19\n\x17org.aiddl.external.grpcb\x06proto3')

_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'receiver_pb2', globals())
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  DESCRIPTOR._serialized_options = b'\n\027org.aiddl.external.grpc'
  _ORDER._serialized_start=267
  _ORDER._serialized_end=310
  _QUERY._serialized_start=57
  _QUERY._serialized_end=204
  _MESSAGES._serialized_start=206
  _MESSAGES._serialized_end=265
  _RECEIVER._serialized_start=312
  _RECEIVER._serialized_end=400
# @@protoc_insertion_point(module_scope)
