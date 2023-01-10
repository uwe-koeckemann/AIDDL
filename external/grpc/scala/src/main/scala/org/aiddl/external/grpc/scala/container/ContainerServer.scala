package org.aiddl.external.grpc.scala.container

import io.grpc.{Server, ServerBuilder}
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.util.StopWatch
import org.aiddl.core.scala.util.logger.Logger
import org.aiddl.external.grpc.container.ResultStatus.SUCCESS
import org.aiddl.external.grpc.container.{ContainerGrpc, FunctionCallRequest, FunctionCallResult}
import org.aiddl.external.grpc.scala.converter.Converter

import scala.concurrent.{ExecutionContext, Future}

object ContainerServer {
  def runAiddlGrpcServer(port: Int, container: Container): Unit = {
    val server = new ContainerServer(ExecutionContext.global, port, container)
    server.start()
    server.blockUntilShutdown()
  }
}


class ContainerServer(executionContext: ExecutionContext, port: Int, container: Container) { self =>
  private[this] var server: Server = null

  val parser = new Parser(container)
  val converter = new Converter(container)

  def start(): Unit = {
    server = ServerBuilder.forPort(port).addService(ContainerGrpc.bindService(new ContainerImpl, executionContext)).build.start
    sys.addShutdownHook {
      System.err.println("*** shutting down gRPC server since JVM is shutting down")
      self.stop()
      System.err.println("*** server shut down")
    }
  }

  def stop(): Unit = {
    if (server != null) {
      server.shutdown()
    }
  }

  def blockUntilShutdown(): Unit = {
    if (server != null) {
      server.awaitTermination()
    }
  }

  private class ContainerImpl() extends ContainerGrpc.Container {

    override def functionCall(req: FunctionCallRequest) = {
      val uri = Sym(req.functionUri)

      val arg = converter.pb2aiddl(req.getArg)

      //println("================================================")
      //println(s"Request: $uri")
      //println("================================================")
      //println(Logger.prettyPrint(arg, 0))

      val answer = container.getFunctionOrPanic(uri)(arg)

      //println("================================================")
      //println(s"Answer")
      //println("================================================")
      //println(Logger.prettyPrint(answer, 0))

      val conv_answer = converter.aiddl2pb(answer)

      //println(StopWatch.summary)

      val reply = FunctionCallResult().withStatus(SUCCESS).withResult(conv_answer)
      Future.successful(reply)
    }
  }
}
