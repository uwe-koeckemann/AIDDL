package org.aiddl.common.scala.optimization.combinatorial

import org.aiddl.core.scala.representation
import org.aiddl.core.scala.representation.{NaN, Num, Term, Tuple}

case class AiddlCostFunction(term: Term) extends CostFunction  {
  override val scope: Tuple = term(0).asTup
  override val arity: Int = scope.length

  override def apply(assignment: Tuple): Num = {
    if assignment.isGround
    then term(1)(assignment).asNum
    else Num(0)
  }
}
