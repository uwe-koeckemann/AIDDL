package org.aiddl.external.scala.coordination_oru.grpc

import io.grpc.{Server, ServerBuilder}
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.{Sym, Term}
import org.aiddl.core.scala.util.StopWatch
import org.aiddl.core.scala.util.logger.Logger
import org.aiddl.external.grpc.actor.{ActorGrpc, Id, State, Status, Supported}
import org.aiddl.external.grpc.container.ResultStatus.SUCCESS
import org.aiddl.external.grpc.container.{ContainerGrpc, FunctionCallRequest, FunctionCallResult}
import org.aiddl.external.grpc.scala.converter.Converter
import org.aiddl.external.grpc.scala.converter.StatusConverter
import org.aiddl.external.grpc.aiddl.Term as PbTerm
import org.aiddl.external.scala.coordination_oru.actor.CoordinationActor
import org.aiddl.external.scala.coordination_oru.coordinator.CoordinatorWrapper
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation

import scala.concurrent.{ExecutionContext, Future}

object CoordinationActorServer {
  def runAiddlGrpcServer(port: Int,
                         container: Container,
                         coordinationWrapper: CoordinatorWrapper): Unit = {
    val server = new CoordinationActorServer(ExecutionContext.global, port, container, coordinationWrapper)
    server.start()
    server.blockUntilShutdown()
  }
}

class CoordinationActorServer(executionContext: ExecutionContext,
                              port: Int,
                              container: Container,
                              coordinationWrapper: CoordinatorWrapper) { self =>
  private[this] var server: Server = null
  val converter = new Converter(container)
  val statusConv = new StatusConverter(container)

  def start(): Unit = {
    server = ServerBuilder.forPort(port).addService(ActorGrpc.bindService(new ActorImpl, executionContext)).build.start
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
    override def isSupported(request: PbTerm): Future[Supported] = {
      val action = self.converter.pb2aiddl(request)
      val result = self.coordinationWrapper.actor.supported(action)
      Future.successful(Supported().withIsSupported(result))
    }
    override def dispatch(request: PbTerm): Future[Status] = {
      val action = self.converter.pb2aiddl(request)
      println(s"DISPATCHING: $action")
      val result = self.coordinationWrapper.actor.dispatch(action) match {
        case Some(id) => {
          Future.successful(statusConv.aiddl2pb(id, self.coordinationWrapper.actor.status(id)))
        }
        case None => {
          Future.successful(Status().withId(-1).withState(State.REJECTED))
        }
      }
      result
    }
    override def getStatus(request: Id): Future[Status] = {
      val id = request.id
      Future.successful(statusConv.aiddl2pb(id, self.coordinationWrapper.actor.status(id)))
    }

    override def cancel(request: Id): Future[Status] = ???
  }
}