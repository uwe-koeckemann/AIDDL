package org.aiddl.external.grpc.scala.actor

import io.grpc.ManagedChannelBuilder
import org.aiddl.common.scala.execution.Actor
import org.aiddl.common.scala.execution.Actor.ActionInstanceId
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.{Sym, Term}
import org.aiddl.external.grpc.aiddl.{Term => PbTerm}
import org.aiddl.external.grpc.actor.{ActorGrpc, Id, State, Status, Supported}
import org.aiddl.external.grpc.scala.converter.Converter

import scala.concurrent.Future

class ActorClient(host: String, port: Int, container: Container) extends Actor {
  val parser = new Parser(container)
  val converter = new Converter(container)

  val channel = ManagedChannelBuilder
    .forAddress(host, port)
    .usePlaintext()
    .build

  val blockingStub = ActorGrpc.blockingStub(channel)

  override def supported(action: Term): Boolean =
    val response: Supported = blockingStub.isSupported(converter.aiddl2pb(action))
    response.isSupported

  override def dispatch(action: Term): Option[ActionInstanceId] = {
    val r = processResponse(blockingStub.dispatch(this.converter.aiddl2pb(action)))
    r match {
      case Some(id) => this.actionIdMap.put(id, action)
      case None => {}
    }
    r
  }

  override def cancel(id: ActionInstanceId): Unit =
    processResponse(blockingStub.cancel(Id(id)))

  override def tick: Unit =
    this.actionIdMap.keys.filter( this.getStatus(_) match {
      case Some(s) => !s.isDone
      case None => false
    }).foreach( id => processResponse(blockingStub.getStatus(Id(id))))

  private def processResponse( newState: Status ): Option[ActionInstanceId] = {
    newState match {
      case Status(id, State.PENDING, _, _, _) => this.update(id, Actor.Status.Pending); Some(id)
      case Status(id, State.ACTIVE, _, _, _) => this.update(id, Actor.Status.Active); Some(id)
      case Status(id, State.SUCCEEDED, _, _, _) => this.update(id, Actor.Status.Succeeded); Some(id)
      case Status(id, State.ERROR, fb, msg, _) => {
        val feedback = fb match {
          case Some(value) => this.converter.pb2aiddl(value)
          case None => Sym("NIL")
        }
        this.update(id, Actor.Status.Error(feedback, msg))
        Some(id)
      }
      case Status(id, State.RECALLING, _, _, _) => this.update(id, Actor.Status.Recalling); Some(id)
      case Status(id, State.RECALLED, _, _, _) => this.update(id, Actor.Status.Recalled); Some(id)
      case Status(id, State.PREEMPTING, _, _, _) => this.update(id, Actor.Status.Preempting); Some(id)
      case Status(id, State.PREEMPTED, _, _, _) => this.update(id, Actor.Status.Preempted); Some(id)
      case Status(_, State.REJECTED, _, _, _) => None
      case _ => ???
    }
  }

}
