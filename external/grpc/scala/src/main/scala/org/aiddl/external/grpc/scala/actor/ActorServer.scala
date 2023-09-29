package org.aiddl.external.grpc.scala.actor

import io.grpc.{Server, ServerBuilder}
import org.aiddl.common.scala.execution.Actor

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.util.StopWatch
import org.aiddl.core.scala.util.logger.Logger
import org.aiddl.external.grpc.actor.{ActorGrpc, Id, Status, Supported, State}
import org.aiddl.external.grpc.aiddl.Term
import org.aiddl.external.grpc.container.ResultStatus.SUCCESS
import org.aiddl.external.grpc.container.{ContainerGrpc, FunctionCallRequest, FunctionCallResult}
import org.aiddl.external.grpc.scala.converter.{Converter, StatusConverter}

import scala.concurrent.{ExecutionContext, Future}
import scala.concurrent.ExecutionContext.Implicits.global

object ActorServer {
  def runAiddlGrpcServer(port: Int, container: Container, actor: Actor): Unit = {
    val server = new ActorServer(ExecutionContext.global, port, container, actor)
    server.start()
    server.blockUntilShutdown()
  }
}

class ActorServer(executionContext: ExecutionContext, port: Int, container: Container, actor: Actor) { self =>
  private[this] var server: Server = null
  val converter = new Converter(container)

  def start(): Unit = {
    val actorImpl = new ActorImpl(actor)
    server = ServerBuilder.forPort(port).addService(ActorGrpc.bindService(actorImpl, executionContext)).build.start
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

  private class ActorImpl(actor: Actor) extends ActorGrpc.Actor {
    private val converter = new Converter(container)
    private val statusConverter = new StatusConverter(container)

    override def isSupported(request: Term): Future[Supported] =
      val aiddlRequest = converter.pb2aiddl(request)
      val response = actor.supported(aiddlRequest)
      Future(Supported(response))

    override def dispatch(request: Term): Future[Status] = {
      val aiddlRequest = converter.pb2aiddl(request)
      val response = actor.dispatch(aiddlRequest)
      val status = response match
        case Some(id) =>
          actor.getStatus(id) match
            case Some(status) => statusConverter.aiddl2pb(id, status)
            case None => {
              val errorMessage = s"Simulator dispatched but assigned no state. This is a bug in the simulator.\n\tAction: $aiddlRequest\n\tAction ID: $id"
              Status().withId(id).withState(State.ERROR).withMsg(errorMessage)
              throw new IllegalStateException(errorMessage)
            }
        case None =>
          Status().withState(State.REJECTED)

      Future(status)
    }

    override def getStatus(request: Id): Future[Status] = {
      val status = actor.getStatus(request.id) match
        case Some(status) =>
          statusConverter.aiddl2pb(request.id, status)
        case None =>
          Status().withId(request.id).withState(State.ERROR).withMsg(s"Unknown action instance ID: ${request.id}")

      Future(status)
    }

    override def cancel(request: Id): Future[Status] = {
      actor.cancel(request.id)
      val status = actor.getStatus(request.id) match
        case Some(status) =>
          statusConverter.aiddl2pb(request.id, status)
        case None =>
          Status().withId(request.id).withState(State.ERROR).withMsg(s"Unknown action instance ID: ${request.id}")
      Future(status)
    }
  }
}
