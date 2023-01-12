package org.aiddl.common.scala.execution.dispatch

import org.aiddl.common.scala.execution.Actor
import org.aiddl.common.scala.execution.Actor.Status.*
import org.aiddl.common.scala.execution.Actor.{ActionInstanceId, Status}
import org.aiddl.common.scala.execution.clock.Tickable
import org.aiddl.core.scala.representation.{Sym, Term}

import scala.collection.immutable.Queue

class QueueDispatcher extends Dispatcher {
  private var queue: Queue[Term] = Queue.empty
  private var current: Option[List[(Actor, ActionInstanceId)]] = None

  private var currentAction: Option[Term] = None

  def abortOnError( id: Term, action: Term, actor: Actor, status: Status) = {
    queue = Queue.empty
  }

  def enqueue( action: Term ) =
    queue = queue.enqueue(action)

  def enqueueAll( actions: Seq[Term] ) =
    queue = queue.enqueueAll(actions)

  override def isIdle: Boolean = current == None && this.queue.isEmpty

  def tick =
    val readyForNext = current match {
      case None => true
      case Some(list) => {
        val remainingActors = list.filter( actInfo => {
          val (actor, instId) = actInfo
          actor.tick
          actor.status(instId) match {
            case Succeeded => false
            case Recalled => false
            case Preempted => false
            case err@Error(_, _) => {
              this.errorHandler(Sym("none"), currentAction.get, actor, err)
              false
            }
            case _ => true
          }
        })
        current = if ( remainingActors.isEmpty ) None else Some(remainingActors)
        current.isEmpty
      }
    }

    if ( readyForNext ) {
      current =
        if ( queue.isEmpty ) { None }
        else {
          val (action, queueNew) = queue.dequeue
          queue = queueNew
          currentAction = Some(action)
          val runningActors = actors
            .map(a => (a, a.dispatch(action)))
            .collect({ case (actor, Some(aId)) => (actor, aId) })
          Some(runningActors)
        }
    }
}
