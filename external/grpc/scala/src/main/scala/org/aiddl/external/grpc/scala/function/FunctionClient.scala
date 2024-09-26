package org.aiddl.external.grpc.scala.function

import io.grpc.ManagedChannelBuilder
import org.aiddl.external.grpc.function.FunctionGrpc
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.Term
import org.aiddl.core.scala.util.StopWatch
import org.aiddl.external.grpc.scala.converter.Converter

class FunctionClient(host: String, port: Int, c: Container) {
  private val converter = new Converter(c)

  private val channel = ManagedChannelBuilder
    .forAddress(host, port)
    .usePlaintext()
    .build

  private val blockingStub = FunctionGrpc.blockingStub(channel)

  def call(x: Term): Term = {
    val answer = blockingStub.call(this.converter.aiddl2pb(x))
    this.converter.pb2aiddl(answer)
  }
}