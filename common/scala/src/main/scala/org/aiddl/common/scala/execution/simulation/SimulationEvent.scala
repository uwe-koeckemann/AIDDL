package org.aiddl.common.scala.execution.simulation

import org.aiddl.core.scala.representation.{Num, Term}

/**
 * An event that can fire during simulation
 */
trait SimulationEvent {

  /**
   * Check if the event is still possible. Used for events that can only occur a limited number of times.
   * @return <code>true</code> if the event can still occur, <code>false</code> otherwise
   */
  def possible: Boolean

  /**
   * Probability of the event occurring if it is applicable.
   * @return probability 0 <= p <= 1
   */
  def probability: Num

  /**
   * Check if the event can occur in the current state.
   * @param state
   * @return <code>true</code> if the event conditions are met, <code>false</code> otherwise
   */
  def applicable(state: Term): Boolean

  /**
   * Apply the event to a state
   * @param state the current state
   * @return the new state
   */
  def apply(state: Term): Term
}
