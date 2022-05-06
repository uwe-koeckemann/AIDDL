package org.aiddl.common.scala.execution.clock

trait Tickable {

  /** Tick an object to update its internal state. This should normally be a cheap operation. */
  def tick: Unit
}
