# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: aiddl.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='aiddl.proto',
  package='aiddl',
  syntax='proto3',
  serialized_options=b'\n\021org.aiddl.network',
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x0b\x61iddl.proto\x12\x05\x61iddl\"8\n\x13\x46unctionCallRequest\x12\x14\n\x0c\x66unction_uri\x18\x01 \x01(\t\x12\x0b\n\x03\x61rg\x18\x02 \x01(\t\"4\n\x12\x46unctionCallResult\x12\x0e\n\x06status\x18\x01 \x01(\x05\x12\x0e\n\x06result\x18\x02 \x01(\t2N\n\x05\x41iddl\x12\x45\n\x0c\x46unctionCall\x12\x1a.aiddl.FunctionCallRequest\x1a\x19.aiddl.FunctionCallResultB\x13\n\x11org.aiddl.networkb\x06proto3'
)




_FUNCTIONCALLREQUEST = _descriptor.Descriptor(
  name='FunctionCallRequest',
  full_name='aiddl.FunctionCallRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='function_uri', full_name='aiddl.FunctionCallRequest.function_uri', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='arg', full_name='aiddl.FunctionCallRequest.arg', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=22,
  serialized_end=78,
)


_FUNCTIONCALLRESULT = _descriptor.Descriptor(
  name='FunctionCallResult',
  full_name='aiddl.FunctionCallResult',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='status', full_name='aiddl.FunctionCallResult.status', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='result', full_name='aiddl.FunctionCallResult.result', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=80,
  serialized_end=132,
)

DESCRIPTOR.message_types_by_name['FunctionCallRequest'] = _FUNCTIONCALLREQUEST
DESCRIPTOR.message_types_by_name['FunctionCallResult'] = _FUNCTIONCALLRESULT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FunctionCallRequest = _reflection.GeneratedProtocolMessageType('FunctionCallRequest', (_message.Message,), {
  'DESCRIPTOR' : _FUNCTIONCALLREQUEST,
  '__module__' : 'aiddl_pb2'
  # @@protoc_insertion_point(class_scope:aiddl.FunctionCallRequest)
  })
_sym_db.RegisterMessage(FunctionCallRequest)

FunctionCallResult = _reflection.GeneratedProtocolMessageType('FunctionCallResult', (_message.Message,), {
  'DESCRIPTOR' : _FUNCTIONCALLRESULT,
  '__module__' : 'aiddl_pb2'
  # @@protoc_insertion_point(class_scope:aiddl.FunctionCallResult)
  })
_sym_db.RegisterMessage(FunctionCallResult)


DESCRIPTOR._options = None

_AIDDL = _descriptor.ServiceDescriptor(
  name='Aiddl',
  full_name='aiddl.Aiddl',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_start=134,
  serialized_end=212,
  methods=[
  _descriptor.MethodDescriptor(
    name='FunctionCall',
    full_name='aiddl.Aiddl.FunctionCall',
    index=0,
    containing_service=None,
    input_type=_FUNCTIONCALLREQUEST,
    output_type=_FUNCTIONCALLRESULT,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
])
_sym_db.RegisterServiceDescriptor(_AIDDL)

DESCRIPTOR.services_by_name['Aiddl'] = _AIDDL

# @@protoc_insertion_point(module_scope)
