package org.aiddl.util.scala.grpc

import io.grpc.ManagedChannelBuilder
import org.aiddl.core.scala.representation.{Sym, Term}
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.parser.Parser
import org.aiddl.util.scala.grpc.generated.aiddl.{AiddlGrpc, FunctionCallRequest, FunctionCallResult}

class GrpcFunction(host: String, port: Int, uri: Sym, parser: Parser) extends Function {

  val channel = ManagedChannelBuilder
    .forAddress(host, port)
    .usePlaintext()
    .build

  val blockingStub = AiddlGrpc.blockingStub(channel)

  override def apply(x: Term): Term = {
    val request = FunctionCallRequest(functionUri = "test-uri", arg = "1")
    val result = blockingStub.functionCall(request)
    val resultAiddl = parser.str(result.result)
    resultAiddl
  }
}
