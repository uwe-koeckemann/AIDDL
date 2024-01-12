package org.aiddl.common.scala.execution

import org.aiddl.common.scala.execution.Actor.Status.Succeeded
import org.aiddl.common.scala.execution.Actor.{ActionInstanceId, Status}
import org.aiddl.common.scala.execution.Controller.Signal.*
import org.aiddl.common.scala.execution.Controller.{Instruction, Signal}
import org.aiddl.common.scala.execution.clock.Tickable
import org.aiddl.common.scala.execution.Actor
import org.aiddl.common.scala.execution.sensor.Sensor
import org.aiddl.core.scala.representation.{Num, Term}

object Controller {
  object Instruction {
    /** Create an instruction without a controller-side ID
     * @param action action to dispatch
     * @return instruction with actionId set to None
     */
    def apply(action: Term): Instruction =
      Instruction(None, action)
  }

  /**
   * Instruction sent out by a controller.
   * The optional actionId can be used to track internal dispatch constraints (e.g., precedence).
   * @param actionId optional, controller assigned, action ID
   * @param action   the action to dispatch
   */
  case class Instruction(actionId: Option[Term], action: Term)

  /** Signals sent to registered callback functions while controller is running. */
  enum Signal {
    /** Controller has been enabled */
    case Enabled
    /** Controller has been disabled */
    case Disabled
    /** Controller skipped tick because it is disabled */
    case Skipped
    /** Controller has reached a goal */
    case GoalReached(goal: Term)
    /** Controller goal has been updated */
    case GoalUpdated(goal: Term)
    /** Controller has dispatched an action */
    case Dispatched(instanceId: ActionInstanceId, actionId: Term, action: Term)
    /** Action succeeded, but sensed state was not expected */
    case UnexpectedActionResult(actionId: Term, action: Term)
    /** Actor failed to execute an action */
    case ActorFailure(instanceId: ActionInstanceId, actionId: Term, action: Term, code: Term, message: String)
    /** An action could not be dispatched */
    case DispatchFailure(id: Option[Term], action: Term)
    /** Controller cannot reach goal from sensed state */
    case GoalUnreachable(goal: Term)
  }
}

/**
 * Controller abstraction.
 */
trait Controller extends Tickable {
  val sensor: Sensor
  val actor: Actor

  private var enabled: Boolean = true
  private var actionIdMap: Map[Term, ActionInstanceId] = Map.empty
  private var actionMap: Map[Term, Term] = Map.empty
  private var goal: Term = _
  private var callbacks: List[Signal => Unit] = Nil

  /**
   * Set the goal for this controller.
   * @param goal
   */
  def setGoal(goal: Term): Unit = {
    this.goal = goal
    callback(GoalUpdated(goal))
  }

  /**
   * Read the current goal the controller is using.
   * @return
   */
  def currentGoal: Term =
    goal

  /**
   * Given a state, decide what to do
   * @param state sensed state
   * @return action if deemed necessary
   */
  def decide(state: Term): List[Instruction]

  /**
   * Enable actor to dispatch decided actions.
   */
  def enable: Unit =
    this.enabled = true
    callback(Enabled)

  /**
   * Disable actor. Ticking it will do nothing after this.
   */
  def disable: Unit =
    this.enabled = false
    callback(Disabled)

  /**
   * Register a new callback function for controller signals
   * @param cb the callback function
   */
  def registerCallback(cb: Signal => Unit): Unit =
    this.callbacks = cb :: this.callbacks

  /**
   * Remove a callback function
   * @param cb the callback function
   */
  def removeCallback(cb: Signal => Unit): Unit =
    this.callbacks = this.callbacks.filter(_ != cb)

  /**
   * Perform callbacks with a given signal
   * @param signal
   */
  protected def callback(signal: Signal): Unit =
    callbacks.foreach(cb => cb(signal))

  /**
   * If enabled, read the most recent sensor state, decide on actions,
   * and send any decided actions to the actor.
   */
  override def tick: Unit = {
    if (enabled) {
      val state = sensor.sense.value
      val instructions = decide(state)
      this.cleanActionStates()
      for ( Instruction(maybeId, action) <- instructions ) {
        actor.dispatch(action) match {
          case Some(actionInstanceId) =>
            val id = maybeId match {
              case Some(value) => value
              case None => Num(actionInstanceId)
            }
            this.actionIdMap = this.actionIdMap.updated(id, actionInstanceId)
            this.actionMap = this.actionMap.updated(id, action)
            callback(Dispatched(actionInstanceId, id, action))
          case None => {
            callback(DispatchFailure(maybeId, action))
            throw new IllegalArgumentException(
              s"Actor ${this.actor.getClass} " +
                s"of controller ${this.getClass.getName} " +
                s"not able to dispatch $action")
          }
        }
      }
    } else {
      callback(Skipped)
    }
  }

  private def cleanActionStates(): Unit = {
    val done = this.actionIdMap
      .filter((actionId, instanceId) => {
        val state = this.actor.status(instanceId)
        state match {
          case Status.Error(code, message) =>
            this.callback(ActorFailure(instanceId, actionId, this.actionMap(actionId), code, message))
          case _ => {}
        }
        state.isDone
      })
      .map((actionId, _) => actionId)

    this.actionIdMap = this.actionIdMap.removedAll(done)
    this.actionMap = this.actionMap.removedAll(done)
  }
}
