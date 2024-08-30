package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.core.scala.representation.*

case class Constraint(term: Tuple) {
  def arity: Int = term(0).length
  def scope: Tuple = term(0).asTup

  def satisfiedBy(assignment: Tuple): Boolean = {
    if term(1).isInstanceOf[CollectionTerm]
    then {
      val r = if assignment.isGround
      then term(1).asCol.contains(assignment)
      else term(1).asCol.exists(x => assignment.unifiable(x))
      r
    } else {
      try {
        term(1)(assignment).boolVal
      } catch {
        case _ => true
      }
    }
  }

  def satisfiedBy(substitution: Substitution): Boolean =
    this.satisfiedBy((scope \ substitution).asTup)

}
