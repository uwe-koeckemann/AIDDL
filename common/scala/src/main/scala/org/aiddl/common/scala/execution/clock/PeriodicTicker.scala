package org.aiddl.common.scala.execution.clock

/**
 * Tick all children every <code>n</code> ticks.
 * @param n number of ticks in the period
 * @param offset offset on which children are ticked
 * @param children controlled tickable instances
 */
class PeriodicTicker(n: Int, offset: Int, children: List[Tickable]) extends Tickable {
  var c = -1

  def tick = {
    c = (c + 1) % n
    if ( c == offset )  children.foreach(_.tick)
  }
}
