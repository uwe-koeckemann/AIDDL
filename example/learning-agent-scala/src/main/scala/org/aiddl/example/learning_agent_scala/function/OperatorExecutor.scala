package org.aiddl.example.learning_agent_scala.function

import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.{CollectionTerm, KeyVal, ListTerm, SetTerm, Sym, Term, Tuple}
import org.aiddl.common.scala.planning.PlanningTerm
import org.aiddl.common.scala.planning.state_variable.{ApplicableFunction, StateTransition}

class OperatorExecutor(operators: CollectionTerm) extends Function with Verbose {
  private val fApplyOperator = new StateTransition()

  override def apply(x: Term): Term = {
    val s = x(PlanningTerm.State).asSet
    val a = x(Sym("action"))
    this.apply(s, a)
  }

  def apply(state: SetTerm, action: Term): SetTerm = {
    var s_next: Term = Sym("NIL")
    for (o <- this.operators) {
      val name = o(PlanningTerm.Name)
      name.unify(action) match {
        case Some(sub) => {
          val o_sub = (o \ sub).asTup
          s_next = fApplyOperator(o_sub, state.asSet)

          this.logger.info("Applying: " + name)
          this.logger.info("  s  = " + state)
          this.logger.info("  s' = " + s_next)
        }
        case None => {}
      }
    }

    if (s_next == Sym("NIL")) {
      this.logger.info("Failure: Action " + action + " not applicable to " + state)
      s_next = state
    } else {
      this.logger.info("Success: " + state + " x " + action + " -> " + s_next)
    }
    s_next.asSet
  }
}
