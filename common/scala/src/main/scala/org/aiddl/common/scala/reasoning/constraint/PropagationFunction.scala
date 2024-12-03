package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.representation._

trait PropagationFunction {

  def init(csp: ConstraintSatisfactionProblem): Unit

  def propagate(assignment: List[Term], domains: Map[Term, Seq[Term]]): Option[Map[Term, Seq[Term]]]
}
