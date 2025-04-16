package org.aiddl.common.scala.optimization.combinatorial

import org.aiddl.core.scala.representation.{Tuple, Num}

trait CostFunction {
  val scope: Tuple
  val arity: Int

  def apply(assignment: Tuple): Num
}
