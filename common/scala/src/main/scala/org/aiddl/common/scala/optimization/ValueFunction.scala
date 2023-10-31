package org.aiddl.common.scala.optimization

import org.aiddl.core.scala.representation.*

trait ValueFunction[S, N] extends Ordering[S] {
  val ordering: Ordering[N]
  def apply(x: S): N

  override def compare(x: S, y: S): Int =
    ordering.compare(this(x), this(y))
}
