package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm.{Constraints, Domains, Variables}
import org.aiddl.common.scala.reasoning.constraint.{ConstraintSatisfactionProblem, CspSolver}
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.*

object NodeConsistency {
  def apply(csp: ConstraintSatisfactionProblem): ConstraintSatisfactionProblem = {
    val unaryConstraints = csp.constraints.filter(_.arity == 1)
    var domains = csp.domains
    var constraints = csp.constraints
    for ( unaryCon <- unaryConstraints ) {
      println(unaryCon)
      val x = unaryCon.scope.head
      val feasible = domains(x).filter(v => {
        println(s"$v works for $unaryCon? ${unaryCon.satisfiedBy(Tuple(v))}")
        unaryCon.satisfiedBy(Tuple(v))
      })
      domains = domains.updated(x, feasible)
    }
    constraints = constraints.removedAll(unaryConstraints)

    ConstraintSatisfactionProblem(
      csp.variables,
      domains,
      constraints
    )
  }
}
