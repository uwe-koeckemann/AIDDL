package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm.{Constraints, Domains, Variables}
import org.aiddl.common.scala.reasoning.constraint.{Constraint, ConstraintSatisfactionProblem, CspSolver}
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{CollectionTerm, KeyVal, ListTerm, SetTerm, Substitution, Term, Tuple, Var}
import org.aiddl.core.scala.util.ComboIterator
import org.aiddl.core.scala.util.logger.Logger

import scala.collection.mutable

class ConvertTablesToBinary {

  def apply(csp: ConstraintSatisfactionProblem): ConstraintSatisfactionProblem = {
    val binaryCollection = new mutable.HashMap[Tuple, CollectionTerm]
      .withDefaultValue(SetTerm.empty)

    for ( c <- csp.constraints ) {
      for {
        i1 <- (0 until c.arity) if c.scope(i1).isInstanceOf[Var]
        i2 <- (0 until c.arity) if i1 != i2 && c.scope(i2).isInstanceOf[Var]
      } {
        val inverse = c.scope(i1).toString > c.scope(i2).toString
        val i = if inverse then i2 else i1
        val j = if inverse then i1 else i2

        val x1 = c.scope(i)
        val x2 = c.scope(j)
        val scope = Tuple(x1, x2)
        val relations =
          SetTerm(c.term(1).asCol.map(row => Tuple(row(i), row(j))).toSet)

        binaryCollection.put(scope, binaryCollection(scope).addAll(relations))
      }
    }

    val binaryConstraints =
      binaryCollection.map((scope, relations) => Constraint(Tuple(scope, relations))).toSet

    new ConstraintSatisfactionProblem(
      csp.variables,
      csp.domains,
      binaryConstraints
    )
  }
}
