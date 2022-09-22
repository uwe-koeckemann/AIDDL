package org.aiddl.util.scala.grpc

import java.util.logging.Logger
import io.grpc.{Server, ServerBuilder}
import org.aiddl.util.scala.grpc.generated.aiddl.{AiddlGrpc, FunctionCallRequest, FunctionCallResult}

import scala.concurrent.{ExecutionContext, Future}

object HelloWorldServer {
  private val logger = Logger.getLogger(classOf[HelloWorldServer].getName)

  def main(args: Array[String]): Unit = {
    val server = new HelloWorldServer(ExecutionContext.global)
    server.start()
    server.blockUntilShutdown()
  }

  private val port = 50051
}


class HelloWorldServer(executionContext: ExecutionContext) { self =>
  private[this] var server: Server = null

  private def start(): Unit = {
    server = ServerBuilder.forPort(HelloWorldServer.port).addService(AiddlGrpc.bindService(new AiddlImpl, executionContext)).build.start
    HelloWorldServer.logger.info("Server started, listening on " + HelloWorldServer.port)
    sys.addShutdownHook {
      System.err.println("*** shutting down gRPC server since JVM is shutting down")
      self.stop()
      System.err.println("*** server shut down")
    }
  }

  private def stop(): Unit = {
    if (server != null) {
      server.shutdown()
    }
  }

  private def blockUntilShutdown(): Unit = {
    if (server != null) {
      server.awaitTermination()
    }
  }

  private class AiddlImpl extends AiddlGrpc.Aiddl {
    override def functionCall(req: FunctionCallRequest) = {
      val reply = FunctionCallResult(status = 0, result = "Result")
      Future.successful(reply)
    }
  }

}
