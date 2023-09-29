package org.aiddl.common.scala.execution

import org.aiddl.common.scala.Common
import org.aiddl.common.scala.execution.clock.Tickable
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{Sym, Term}

object Sensor {
  type SeqId = Long
}

trait Sensor extends Tickable {
  import Sensor.SeqId

  protected var currentValue: Term = Common.NIL
  protected var currentSequenceId: SeqId = 0

  var callbacks: List[(SeqId, Term) => Unit] = Nil

  /**
   * Sense
   * @return
   */
  def sense: Term

  /**
   * Get latest sequence ID and sensor value
   * @return
   */
  def latest: (SeqId, Term) = (this.currentSequenceId, this.currentValue)

  /**
   * Get latest sequence ID
   * @return
   */
  def latestSequenceId: SeqId = this.currentSequenceId

  /**
   * Get latest sensed value
   * @return
   */
  def latestValue: Term = this.currentValue

  def registerCallback( f: (SeqId, Term) => Unit ): Unit =
    callbacks = f :: callbacks

  /**
   * Sense a new value, update latest value and sequence ID and perform callbacks
   */
  override def tick = {
    this.currentValue = this.sense
    this.currentSequenceId += 1
    callbacks.foreach( f => f(this.currentSequenceId, this.currentValue) )
  }
}
