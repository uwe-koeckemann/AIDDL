package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.*

class AllDifferentConstraint(n: Int) extends Function {
  override def apply(x: Term): Term =
    Bool(x.asTup.toSet.size == n)
}
