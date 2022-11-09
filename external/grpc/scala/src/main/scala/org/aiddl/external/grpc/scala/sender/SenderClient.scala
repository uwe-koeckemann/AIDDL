package org.aiddl.external.grpc.scala.sender

import io.grpc.ManagedChannelBuilder
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.Term
import org.aiddl.external.grpc.aiddl.AiddlStr
import org.aiddl.external.grpc.scala.converter.Converter
import org.aiddl.external.grpc.sender.SenderGrpc

class SenderClient(host: String, port: Int, container: Container) {
  val converter = new Converter(container)
  val channel = ManagedChannelBuilder
    .forAddress(host, port)
    .usePlaintext()
    .build

  val blockingStub = SenderGrpc.blockingStub(channel)

  def send(x: Term): Unit = {
    blockingStub.send(this.converter.aiddl2pb(x))
  }

}
