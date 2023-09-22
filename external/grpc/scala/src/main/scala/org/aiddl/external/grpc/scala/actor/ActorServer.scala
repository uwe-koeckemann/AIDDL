package org.aiddl.external.grpc.scala.actor

import io.grpc.{Server, ServerBuilder}
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.util.StopWatch
import org.aiddl.core.scala.util.logger.Logger
import org.aiddl.external.grpc.actor.{ActorGrpc, Id, Status, Supported}
import org.aiddl.external.grpc.aiddl.Term
import org.aiddl.external.grpc.container.ResultStatus.SUCCESS
import org.aiddl.external.grpc.container.{ContainerGrpc, FunctionCallRequest, FunctionCallResult}
import org.aiddl.external.grpc.scala.converter.Converter

import scala.concurrent.{ExecutionContext, Future}

object ActorServer {
  def runAiddlGrpcServer(port: Int, container: Container, actor: ActorGrpc.Actor): Unit = {
    val server = new ActorServer(ExecutionContext.global, port, container, actor)
    server.start()
    server.blockUntilShutdown()
  }
}

class ActorServer(executionContext: ExecutionContext, port: Int, container: Container, actor: ActorGrpc.Actor) { self =>
  private[this] var server: Server = null
  val converter = new Converter(container)

  def start(): Unit = {
    server = ServerBuilder.forPort(port).addService(ActorGrpc.bindService(actor, executionContext)).build.start
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

  private class ActorImpl() extends ActorGrpc.Actor {
    override def isSupported(request: Term): Future[Supported] = ???
    override def dispatch(request: Term): Future[Status] = ???
    override def getStatus(request: Id): Future[Status] = ???
    override def cancel(request: Id): Future[Status] = ???
  }
}
