package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm.{Constraints, Domains, Variables}
import org.aiddl.core.scala.representation.*

import scala.collection.mutable
import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_KeyVal

import scala.language.implicitConversions


object ConstraintSatisfactionProblem {
  def fromTerm(csp: Term): ConstraintSatisfactionProblem = {
    ConstraintSatisfactionProblem(
      csp(Variables).asList.toList,
      csp(Domains).asCol.map( kvp => kvp.key -> kvp.value.asList.toList ).toMap,
      csp(Constraints).asCol.map( c => AiddlConstraint(c.asTup) ).toSet
    )
  }
}

class ConstraintSatisfactionProblem(
                                     var variables: Seq[Term],
                                     var domains: Map[Term, Seq[Term]],
                                     var constraints: Set[Constraint]
                                   ) {
  val constraintMap = new mutable.HashMap[Term, Set[Constraint]]().withDefaultValue(Set.empty)

  constraints.foreach(c => {
    c.scope.asTup.filter(_.isInstanceOf[Var]).foreach(x => constraintMap.put(x, constraintMap(x) + c))
  })

  override def toString: String = {
    s"X = { ${variables.mkString(", ")} }\n" +
    s"D = { ${domains.mkString("\n\t", "\n\t", "\n")}}\n" +
    s"C = { ${constraints.mkString("\n\t", "\n\t", "\n")}}\n"
  }
}
