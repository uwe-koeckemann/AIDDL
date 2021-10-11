package org.aiddl.common.scala.execution

trait Timestamped {
  def markNow: Unit
  def markStamp( ts: Int ): Unit
  def lastStamp: Int
}
