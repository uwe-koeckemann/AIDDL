# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: aiddl.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0b\x61iddl.proto\x12\x17org.aiddl.external.grpc\"2\n\x08Rational\x12\x11\n\tnominator\x18\x01 \x01(\x03\x12\x13\n\x0b\x64\x65nominator\x18\x02 \x01(\x03\"w\n\x0e\x43ollectionTerm\x12\x38\n\x07\x63olType\x18\x01 \x01(\x0e\x32\'.org.aiddl.external.grpc.CollectionType\x12+\n\x04\x64\x61ta\x18\x02 \x03(\x0b\x32\x1d.org.aiddl.external.grpc.Term\"b\n\x06KeyVal\x12*\n\x03key\x18\x01 \x01(\x0b\x32\x1d.org.aiddl.external.grpc.Term\x12,\n\x05value\x18\x02 \x01(\x0b\x32\x1d.org.aiddl.external.grpc.Term\"\x92\x01\n\x06\x45ntRef\x12-\n\x06module\x18\x01 \x01(\x0b\x32\x1d.org.aiddl.external.grpc.Term\x12+\n\x04name\x18\x02 \x01(\x0b\x32\x1d.org.aiddl.external.grpc.Term\x12,\n\x05\x61lias\x18\x03 \x01(\x0b\x32\x1d.org.aiddl.external.grpc.Term\"\xa3\x03\n\x04Term\x12\r\n\x03sym\x18\x01 \x01(\tH\x00\x12\r\n\x03var\x18\x02 \x01(\tH\x00\x12\r\n\x03str\x18\x03 \x01(\tH\x00\x12\x11\n\x07\x62oolean\x18\x04 \x01(\x08H\x00\x12\r\n\x03int\x18\x05 \x01(\x03H\x00\x12\x0e\n\x04real\x18\x06 \x01(\x01H\x00\x12\x35\n\x08rational\x18\x07 \x01(\x0b\x32!.org.aiddl.external.grpc.RationalH\x00\x12\x42\n\x0fother_numerical\x18\x08 \x01(\x0e\x32\'.org.aiddl.external.grpc.OtherNumericalH\x00\x12=\n\ncollection\x18\t \x01(\x0b\x32\'.org.aiddl.external.grpc.CollectionTermH\x00\x12.\n\x03kvp\x18\n \x01(\x0b\x32\x1f.org.aiddl.external.grpc.KeyValH\x00\x12\x32\n\x07\x65nt_ref\x18\x0b \x01(\x0b\x32\x1f.org.aiddl.external.grpc.EntRefH\x00\x12\x11\n\x07\x66un_ref\x18\x0c \x01(\tH\x00\x42\x0b\n\tterm_type\"\x1d\n\x08\x41iddlStr\x12\x11\n\taiddl_str\x18\x01 \x01(\t*3\n\x0eOtherNumerical\x12\x0b\n\x07INF_POS\x10\x00\x12\x0b\n\x07INF_NEG\x10\x01\x12\x07\n\x03NAN\x10\x02*.\n\x0e\x43ollectionType\x12\x08\n\x04LIST\x10\x00\x12\x07\n\x03SET\x10\x01\x12\t\n\x05TUPLE\x10\x02\x42\x19\n\x17org.aiddl.external.grpcb\x06proto3')

_OTHERNUMERICAL = DESCRIPTOR.enum_types_by_name['OtherNumerical']
OtherNumerical = enum_type_wrapper.EnumTypeWrapper(_OTHERNUMERICAL)
_COLLECTIONTYPE = DESCRIPTOR.enum_types_by_name['CollectionType']
CollectionType = enum_type_wrapper.EnumTypeWrapper(_COLLECTIONTYPE)
INF_POS = 0
INF_NEG = 1
NAN = 2
LIST = 0
SET = 1
TUPLE = 2


_RATIONAL = DESCRIPTOR.message_types_by_name['Rational']
_COLLECTIONTERM = DESCRIPTOR.message_types_by_name['CollectionTerm']
_KEYVAL = DESCRIPTOR.message_types_by_name['KeyVal']
_ENTREF = DESCRIPTOR.message_types_by_name['EntRef']
_TERM = DESCRIPTOR.message_types_by_name['Term']
_AIDDLSTR = DESCRIPTOR.message_types_by_name['AiddlStr']
Rational = _reflection.GeneratedProtocolMessageType('Rational', (_message.Message,), {
  'DESCRIPTOR' : _RATIONAL,
  '__module__' : 'aiddl_pb2'
  # @@protoc_insertion_point(class_scope:org.aiddl.external.grpc.Rational)
  })
_sym_db.RegisterMessage(Rational)

CollectionTerm = _reflection.GeneratedProtocolMessageType('CollectionTerm', (_message.Message,), {
  'DESCRIPTOR' : _COLLECTIONTERM,
  '__module__' : 'aiddl_pb2'
  # @@protoc_insertion_point(class_scope:org.aiddl.external.grpc.CollectionTerm)
  })
_sym_db.RegisterMessage(CollectionTerm)

KeyVal = _reflection.GeneratedProtocolMessageType('KeyVal', (_message.Message,), {
  'DESCRIPTOR' : _KEYVAL,
  '__module__' : 'aiddl_pb2'
  # @@protoc_insertion_point(class_scope:org.aiddl.external.grpc.KeyVal)
  })
_sym_db.RegisterMessage(KeyVal)

EntRef = _reflection.GeneratedProtocolMessageType('EntRef', (_message.Message,), {
  'DESCRIPTOR' : _ENTREF,
  '__module__' : 'aiddl_pb2'
  # @@protoc_insertion_point(class_scope:org.aiddl.external.grpc.EntRef)
  })
_sym_db.RegisterMessage(EntRef)

Term = _reflection.GeneratedProtocolMessageType('Term', (_message.Message,), {
  'DESCRIPTOR' : _TERM,
  '__module__' : 'aiddl_pb2'
  # @@protoc_insertion_point(class_scope:org.aiddl.external.grpc.Term)
  })
_sym_db.RegisterMessage(Term)

AiddlStr = _reflection.GeneratedProtocolMessageType('AiddlStr', (_message.Message,), {
  'DESCRIPTOR' : _AIDDLSTR,
  '__module__' : 'aiddl_pb2'
  # @@protoc_insertion_point(class_scope:org.aiddl.external.grpc.AiddlStr)
  })
_sym_db.RegisterMessage(AiddlStr)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  DESCRIPTOR._serialized_options = b'\n\027org.aiddl.external.grpc'
  _OTHERNUMERICAL._serialized_start=915
  _OTHERNUMERICAL._serialized_end=966
  _COLLECTIONTYPE._serialized_start=968
  _COLLECTIONTYPE._serialized_end=1014
  _RATIONAL._serialized_start=40
  _RATIONAL._serialized_end=90
  _COLLECTIONTERM._serialized_start=92
  _COLLECTIONTERM._serialized_end=211
  _KEYVAL._serialized_start=213
  _KEYVAL._serialized_end=311
  _ENTREF._serialized_start=314
  _ENTREF._serialized_end=460
  _TERM._serialized_start=463
  _TERM._serialized_end=882
  _AIDDLSTR._serialized_start=884
  _AIDDLSTR._serialized_end=913
# @@protoc_insertion_point(module_scope)
