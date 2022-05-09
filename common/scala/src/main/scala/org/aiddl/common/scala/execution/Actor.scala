package org.aiddl.common.scala.execution

import org.aiddl.common.scala.execution.Actor
import org.aiddl.common.scala.execution.clock.Tickable
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{Sym, Term}

import scala.collection.mutable

object Actor {
  enum Status {
    case Waiting
    case Running
    case Finished
    case Error(code: Term, msg: String)
    case Aborted

    def isDone: Boolean = this match {
      case Finished | Aborted | Error(_, _) => true
      case _ => false
    }
  }
  type ActionInstanceId = Long
}

trait Actor extends Tickable {
  import Actor.Status.*
  import Actor.{ActionInstanceId, Status}

  protected val actionIdMap: mutable.Map[ActionInstanceId, Term] = new mutable.HashMap[ActionInstanceId, Term]()

  private val stateMap: mutable.Map[ActionInstanceId, Status] = new mutable.HashMap[ActionInstanceId, Status]()
  private var callbacks: List[(ActionInstanceId, Term, Status) => Unit] = Nil
  private var nextFreeId = 0

  /** Check if an action is supported
   *
   * @return `true` if action can be dispatched by this dispatcher, `false` otherwise
   */
  def supported(action: Term): Boolean

  /** Dispatch an action and return a dispatch ID.
   *
   * @return If an action was dispatched a dispatch ID is returned wrapped in an Option as `Some(id)`.
   *         If the action could not be dispatched (e.g., if the dispatcher is busy and does not
   *         support queueing, or if the action is not supported) `None` is returned.
   */
  def dispatch(action: Term): Option[ActionInstanceId]


  /** Dispatch an action and wait for result. Returns only when action has `Done` or `Failed` status.
   *
   * @return If an action was dispatched its ID is returned wrapped in an Option as `Some(id)`.
   *         If the action could not be dispatched (e.g., if the dispatcher is busy and does not
   *         support queueing, or if the action is not supported) `None` is returned.
   */
  def dispatchBlock(action: Term): Option[ActionInstanceId] = {
    this.dispatch(action) match {
      case Some(id) => {
        while {
          !this.status(id).isDone
        } do {
          tick
        }
        Some(id)
      }
      case None => None
    }
  }

  /**
   * Attempt to abort action (if supported). Successful abortion should result in state Aborted
   */
  def abort(id: ActionInstanceId) = {}

  /** Get the status of a dispatch ID if it exists.
   *
   * @return If the ID exists, its latest status message is returned wrapped in an `Option`.
   *         Otherwise, `None` is returned.
   */
  def getStatus(id: ActionInstanceId): Option[Status] = this.stateMap.get(id)

  /** Get the status of a dispatch ID if it exists.
   *
   * @return the latest status message.
   */
  def status(id: ActionInstanceId): Status = this.stateMap(id)

  protected def nextId: ActionInstanceId = {
    val id = nextFreeId
    nextFreeId += 1
    id
  }

  /** Check if dispatcher is idle
   *
   * @return `true` if dispatcher is idle, `false` otherwise
   */
  def idle: Boolean = stateMap.forall( (_, s) => s match {
    case Finished | Error(_, _) => true
    case _ => false
  })

  protected def update(id: ActionInstanceId, s: Status) =
    this.stateMap.get(id) match {
      case Some(currentState) => {
        if (s != currentState) {
          this.stateMap.put(id, s)
          callbacks.foreach(f => f(id, this.actionIdMap.getOrElse(id, Sym("UNKNOWN")), s))
        }
      }
      case None => {
        this.stateMap.put(id, s)
        callbacks.foreach(f => f(id, this.actionIdMap.getOrElse(id, Sym("UNKNOWN")), s))
      }
    }

  /** Register a callback function for this dispatcher. For each status change, every registered callback is called
   * with the dispatch ID and the new status.
   */
  def registerCallback( f: (ActionInstanceId, Term, Status) => Unit ): Unit =
    callbacks = f :: callbacks
}


