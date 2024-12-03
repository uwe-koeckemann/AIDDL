package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.common.scala.reasoning.constraint.{Constraint, CspSolver}
import org.aiddl.core.scala.representation.*

object Revise {
  def apply(domains: Map[Term, Seq[Term]], constraint: Constraint): Option[Map[Term, Seq[Term]]] = {
    var delete = false
    val x = constraint.scope(0)
    val y = constraint.scope(1)

    var newDomainX = domains(x)

    for (v_x <- domains(x)) {
      if (domains(y).forall(v_y => {
        !constraint.satisfiedBy(Tuple(v_x, v_y))
      })) {
        newDomainX = newDomainX.filter(_ != v_x)
        delete = true
      }
    }

    if delete
    then Some(domains.updated(x, newDomainX))
    else None
  }
}
