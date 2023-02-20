package org.aiddl.external.grpc.scala.converter

import org.aiddl.common.scala.execution.Actor.{ActionInstanceId, Status}
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.Num
import org.aiddl.external.grpc.actor.State as PbState
import org.aiddl.external.grpc.actor.Status as PbStatus

class StatusConverter(container: Container) {
  private val converter = new Converter(container)

  def pb2aiddl(state: PbStatus): Status = {
    state.state match {
      case PbState.PENDING => Status.Pending
      case PbState.ACTIVE => Status.Active
      case PbState.SUCCEEDED => Status.Succeeded
      case PbState.ERROR => Status.Error(converter.pb2aiddl(state.getFeedback), state.msg)
      case PbState.RECALLING => Status.Recalling
      case PbState.RECALLED => Status.Recalled
      case PbState.PREEMPTING => Status.Preempting
      case PbState.PREEMPTED => Status.Preempted
      case PbState.REJECTED => Status.Error(converter.pb2aiddl(state.getFeedback), state.msg)
      case PbState.Unrecognized(_) => ???
    }
  }

  def aiddl2pb(id: ActionInstanceId, status: Status): PbStatus = {
    PbStatus()
      .withId(id)
      .withState(
        status match {
          case Status.Pending => PbState.PENDING
          case Status.Active => PbState.ACTIVE
          case Status.Succeeded => PbState.SUCCEEDED
          case Status.Error(_, _) => PbState.ERROR
          case Status.Recalling => PbState.RECALLING
          case Status.Recalled => PbState.RECALLED
          case Status.Preempting => PbState.PREEMPTING
          case Status.Preempted => PbState.PREEMPTED
        })
      .withFeedback(
        status match {
          case Status.Error(code, _) => converter.aiddl2pb(code)
        })
      .withMsg(
        status match {
          case Status.Error(_, msg) => msg
        })
  }
}
