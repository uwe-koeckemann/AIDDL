package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm.{Constraints, Domains, Variables}
import org.aiddl.common.scala.reasoning.constraint.{AiddlConstraint, ConstraintSatisfactionProblem, CspSolver}
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{KeyVal, ListTerm, SetTerm, Substitution, Term, Tuple, Var}
import org.aiddl.core.scala.util.ComboIterator
import org.aiddl.core.scala.util.logger.Logger

import java.util.logging.Level


class ConvertConstraintsToTables {
  private var solver = new CspSolver
  def apply(csp: ConstraintSatisfactionProblem): ConstraintSatisfactionProblem = {
    val domains = csp.domains
    val tables: Set[Constraint] = csp.constraints.map(constraint => {
      val scope = constraint.scope
      val subCsp =
        ConstraintSatisfactionProblem(
          scope.filter(_.isInstanceOf[Var]),
          domains.filter( (x, d) => scope.contains(x) ),
          Set(constraint))
      solver = new CspSolver
      solver.init(subCsp)
      val validArgsTable: Set[Term] = solver.map(valid => {
        val sub = Substitution.from(ListTerm(valid.toList))
        (scope \ sub).asTup
      }).toSet

      AiddlConstraint(Tuple(scope, SetTerm(validArgsTable)))
    })

    ConstraintSatisfactionProblem(
      csp.variables,
      csp.domains,
      tables
    )
  }
}
