package org.aiddl.common.scala.reasoning.constraint
import org.aiddl.core.scala.representation.{CollectionTerm, KeyVal, ListTerm, Substitution, Term, Var}

import scala.collection.mutable
import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm.*
import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_KeyVal

import scala.language.implicitConversions

class ForwardChecking extends PropagationFunction {
  private var csp: ConstraintSatisfactionProblem = _

  def init(csp: ConstraintSatisfactionProblem): Unit = {
    this.csp = csp
  }

  override def propagate(assignment: List[Term], domains: Map[Term, Seq[Term]]): Option[Map[Term, Seq[Term]]] = {
    val sub = new Substitution()
    assignment.foreach(a => sub.add(a.key, a.value))
    var emptyDomain = false
    val newDomains = csp.variables.filter(x => !assignment.exists(y => x == y.key)).map(x => {
      val newDomain = ListTerm(domains(x).filter(v => {
        val sub_x = new Substitution()
        sub_x.add(x, v)
        csp.constraintMap(x).intersect(csp.constraintMap(assignment.head.key)).forall(c => {
          val args = (c.scope \ sub) \ sub_x
          c.satisfiedBy(args.asTup)
        })
      }).toVector)
      if (newDomain.length == 0) emptyDomain = true
      x -> newDomain
    }).toMap
    if (!emptyDomain) {
      Some(newDomains)
    } else {
      None
    }
  }
}
