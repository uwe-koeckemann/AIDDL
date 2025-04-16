package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm.{Constraints, Domains, Variables}
import org.aiddl.common.scala.reasoning.constraint.{AiddlConstraint, ConstraintSatisfactionProblem, CspSolver}
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.*

import scala.collection.mutable


class ArcConsistency3 {

  def apply(csp: ConstraintSatisfactionProblem): ConstraintSatisfactionProblem = {
    val queueSet: mutable.Set[Constraint] = mutable.Set.empty
    queueSet.addAll(csp.constraints.filter(_.arity == 2))

    var domains: Map[Term, Seq[Term]] = csp.domains

    while (queueSet.nonEmpty) {
      val constraint = queueSet.head
      queueSet.remove(constraint)

      Revise(domains, constraint).foreach(newDomains => {
        domains = newDomains
        val toCheckAgain = csp.constraintMap(constraint.scope(0)).filter(c => {
          c.arity == 2
            && c.scope(0) != constraint.scope(0)
            && c.scope(0) != constraint.scope(1)
        })
        queueSet.addAll(toCheckAgain)
      })
    }

    ConstraintSatisfactionProblem(
      csp.variables,
      domains,
      csp.constraints
    )
  }
}
