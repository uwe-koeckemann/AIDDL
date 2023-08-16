package org.aiddl.example.learning_agent_scala.function

import org.aiddl.common.scala.Common
import org.aiddl.common.scala.planning.PlanningTerm
import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.{KeyVal, ListTerm, Sym, Term, Tuple}

import scala.util.Random

class ActionSelector(val basicActions: ListTerm) extends Function with Verbose {
  val r = Random

  override def apply(x: Term): Term = {
    var plan = x
    val selectedAction =
      if (plan == Common.NIL) {
        val randomAction = basicActions(r.nextInt(basicActions.size))
        randomAction
      } else {
        val selectedAction = plan.asList.head
        plan = ListTerm(plan.asList.tail)
        selectedAction
      }

    ListTerm(
      KeyVal(Sym("selected-action"), selectedAction),
      KeyVal(Sym("plan-tail"), plan)
    )
  }
}
