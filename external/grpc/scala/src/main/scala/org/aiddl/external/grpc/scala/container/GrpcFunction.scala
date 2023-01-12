package org.aiddl.external.grpc.scala.container

import io.grpc.ManagedChannelBuilder
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.{Sym, Term}
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.util.StopWatch
import org.aiddl.external.grpc.container.ResultStatus.SUCCESS
import org.aiddl.external.grpc.container.{ContainerGrpc, FunctionCallRequest, FunctionCallResult}
import org.aiddl.external.grpc.scala.converter.Converter

class GrpcFunction(host: String, port: Int, uri: Sym, c: Container) extends Function {
  val converter = new Converter(c)
  val parser = new Parser(c)

  val channel = ManagedChannelBuilder
    .forAddress(host, port)
    .usePlaintext()
    .build

  val blockingStub = ContainerGrpc.blockingStub(channel)

  override def apply(x: Term): Term = {

    StopWatch.start("Convert input")
    val xPb = this.converter.aiddl2pb(x)
    StopWatch.stop("Convert input")

    val request = FunctionCallRequest()
      .withFunctionUri(uri.toString)
      .withArg(xPb)

    StopWatch.start("Calling stub")
    val result = blockingStub.functionCall(request)
    StopWatch.stop("Calling stub")

    val r = result match {
      case FunctionCallResult(SUCCESS, Some(pbTerm), _) => {
        StopWatch.start("Convert output")
        val resultAiddl = this.converter.pb2aiddl(pbTerm)
        StopWatch.stop("Convert output")
        resultAiddl
      }
      case FunctionCallResult(_, _, _) => Sym("NIL")
    }
    r
  }
}
