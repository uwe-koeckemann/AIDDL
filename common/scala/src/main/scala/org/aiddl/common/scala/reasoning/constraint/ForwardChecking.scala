package org.aiddl.common.scala.reasoning.constraint
import org.aiddl.core.scala.representation.{CollectionTerm, KeyVal, ListTerm, Substitution, Term, Var}

import scala.collection.mutable
import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm.*
import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_KeyVal

import scala.language.implicitConversions

class ForwardChecking extends PropagationFunction {
  private var constraintMap = new mutable.HashMap[Term, Set[Term]]().withDefaultValue(Set.empty)
  private var vars: CollectionTerm = _

  override def init(csp: Term): Unit = {
    constraintMap = new mutable.HashMap[Term, Set[Term]]().withDefaultValue(Set.empty)
    csp(Constraints).asCol.foreach(c => {
      val scope = c(0)
      scope.asTup.filter(_.isInstanceOf[Var]).foreach(x => constraintMap.put(x, constraintMap(x) + c))
    })
    vars = csp(Variables).asCol
  }

  override def propagate(assignment: List[Term], domains: CollectionTerm): Option[CollectionTerm] = {
    val sub = new Substitution()
    assignment.foreach(a => sub.add(a.key, a.value))
    var emptyDomain = false
    val newDomains = ListTerm(vars.filter(x => !assignment.exists(y => x == y.key)).map(x => {
      val newDomain = ListTerm(domains(x).asCol.filter(v => {
        val sub_x = new Substitution()
        sub_x.add(x, v)
        constraintMap(x).intersect(constraintMap(assignment.head.key)).forall(c => {
          val args = (c(0) \ sub) \ sub_x
          val pCon = c(1)
          CspSolver.checkConstraint(pCon, args)
        })
      }).toVector)
      if (newDomain.length == 0) emptyDomain = true
      KeyVal(x, newDomain)
    }).toList)
    if (!emptyDomain) {
      Some(newDomains)
    } else {
      None
    }
  }
}
