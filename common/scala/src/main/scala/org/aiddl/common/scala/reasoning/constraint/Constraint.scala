package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.core.scala.representation.Substitution
import org.aiddl.core.scala.representation.Tuple

trait Constraint {
  val arity: Int
  val scope: Tuple

  def satisfiedBy(assignment: Tuple): Boolean

  def satisfiedBy(substitution: Substitution): Boolean =
    this.satisfiedBy((scope \ substitution).asTup)
}
