# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: actor.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from . import aiddl_pb2 as aiddl__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0b\x61\x63tor.proto\x12\x17org.aiddl.external.grpc\x1a\x0b\x61iddl.proto\"\x81\x01\n\x06Status\x12\n\n\x02id\x18\x01 \x01(\x04\x12-\n\x05state\x18\x02 \x01(\x0e\x32\x1e.org.aiddl.external.grpc.State\x12/\n\x08\x66\x65\x65\x64\x62\x61\x63k\x18\x03 \x01(\x0b\x32\x1d.org.aiddl.external.grpc.Term\x12\x0b\n\x03msg\x18\x04 \x01(\t\"\x10\n\x02Id\x12\n\n\x02id\x18\x01 \x01(\x04\"!\n\tSupported\x12\x14\n\x0cis_supported\x18\x01 \x01(\x08*\x84\x01\n\x05State\x12\x0b\n\x07PENDING\x10\x00\x12\n\n\x06\x41\x43TIVE\x10\x01\x12\r\n\tSUCCEEDED\x10\x02\x12\t\n\x05\x45RROR\x10\x03\x12\r\n\tRECALLING\x10\x04\x12\x0c\n\x08RECALLED\x10\x05\x12\x0e\n\nPREEMPTING\x10\x06\x12\r\n\tPREEMPTED\x10\x07\x12\x0c\n\x08REJECTED\x10\x08\x32\xb8\x02\n\x05\x41\x63tor\x12P\n\x0bIsSupported\x12\x1d.org.aiddl.external.grpc.Term\x1a\".org.aiddl.external.grpc.Supported\x12J\n\x08\x44ispatch\x12\x1d.org.aiddl.external.grpc.Term\x1a\x1f.org.aiddl.external.grpc.Status\x12I\n\tGetStatus\x12\x1b.org.aiddl.external.grpc.Id\x1a\x1f.org.aiddl.external.grpc.Status\x12\x46\n\x06\x43\x61ncel\x12\x1b.org.aiddl.external.grpc.Id\x1a\x1f.org.aiddl.external.grpc.StatusB\x19\n\x17org.aiddl.external.grpcb\x06proto3')

_STATE = DESCRIPTOR.enum_types_by_name['State']
State = enum_type_wrapper.EnumTypeWrapper(_STATE)
PENDING = 0
ACTIVE = 1
SUCCEEDED = 2
ERROR = 3
RECALLING = 4
RECALLED = 5
PREEMPTING = 6
PREEMPTED = 7
REJECTED = 8


_STATUS = DESCRIPTOR.message_types_by_name['Status']
_ID = DESCRIPTOR.message_types_by_name['Id']
_SUPPORTED = DESCRIPTOR.message_types_by_name['Supported']
Status = _reflection.GeneratedProtocolMessageType('Status', (_message.Message,), {
  'DESCRIPTOR' : _STATUS,
  '__module__' : 'actor_pb2'
  # @@protoc_insertion_point(class_scope:org.aiddl.external.grpc.Status)
  })
_sym_db.RegisterMessage(Status)

Id = _reflection.GeneratedProtocolMessageType('Id', (_message.Message,), {
  'DESCRIPTOR' : _ID,
  '__module__' : 'actor_pb2'
  # @@protoc_insertion_point(class_scope:org.aiddl.external.grpc.Id)
  })
_sym_db.RegisterMessage(Id)

Supported = _reflection.GeneratedProtocolMessageType('Supported', (_message.Message,), {
  'DESCRIPTOR' : _SUPPORTED,
  '__module__' : 'actor_pb2'
  # @@protoc_insertion_point(class_scope:org.aiddl.external.grpc.Supported)
  })
_sym_db.RegisterMessage(Supported)

_ACTOR = DESCRIPTOR.services_by_name['Actor']
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  DESCRIPTOR._serialized_options = b'\n\027org.aiddl.external.grpc'
  _STATE._serialized_start=239
  _STATE._serialized_end=371
  _STATUS._serialized_start=54
  _STATUS._serialized_end=183
  _ID._serialized_start=185
  _ID._serialized_end=201
  _SUPPORTED._serialized_start=203
  _SUPPORTED._serialized_end=236
  _ACTOR._serialized_start=374
  _ACTOR._serialized_end=686
# @@protoc_insertion_point(module_scope)
