package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.core.scala.representation.*

/**
 * Constraint defined as an AIDDL term (collection of legal assignments or reference to Boolean function)
 * @param term: AIDDL term of form ((?x ?y) C), where (?x ?y) is the scope and C is either a collection term or a function reference term expected to evaluate to a Boolean term
 */
case class AiddlConstraint(term: Tuple) extends Constraint {
  val arity: Int = term.head.length
  val scope: Tuple = term.head.asTup

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
}
