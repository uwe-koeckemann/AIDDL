package org.aiddl.common.scala.reasoning.logic.horn

import org.aiddl.core.scala.representation.{ListTerm, Substitution}

case class ProofStep(goals: ListTerm, substitution: Substitution) {
  override def toString: String =
    if goals.isEmpty
    then substitution.toString()
    else s"${goals.toString()}\n$substitution"
}
