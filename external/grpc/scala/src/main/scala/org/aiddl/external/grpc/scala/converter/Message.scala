package org.aiddl.external.scala.grpc.converter

sealed abstract class Parameter
final case class SingleParameter(typeName: String, parameterName: String) extends Parameter
final case class RepeatedParameter(typeName: String, parameterName: String) extends Parameter
final case class OneOf(parameterName: String, alternatives: List[Parameter]) extends Parameter

case class Message(name: String, params: List[Parameter])

case class Rpc(name: String, inType: String, outType: String)

case class Service(name: String, rpcs: List[Rpc])