package org.aiddl.example.learning_agent_scala.function

import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.{CollectionTerm, KeyVal, ListTerm, SetTerm, Sym, Term, Tuple}
import org.aiddl.common.scala.planning.PlanningTerm
import org.aiddl.common.scala.planning.state_variable.ApplicableFunction

class OperatorExecutor extends Function with Verbose {

  private val fApplyOperator = new ApplicableFunction()

  override def apply(x: Term): Term = {
    val s = x(PlanningTerm.State)
    val a = x(Sym("action"))
    val O = x(PlanningTerm.Operators).asCol

    var s_next: Term = Sym("NIL")

    for (o <- O) {
      val name = o(PlanningTerm.Name)
      name.unify(name) match {
        case Some(sub) => {
          val o_sub = (o \ sub).asTup
          s_next = fApplyOperator(o_sub, s.asSet)

          this.logger.info("Applying: " + name)
          this.logger.info("  s  = " + s)
          this.logger.info("  s' = " + s_next)
        }
        case None => { }
      }
    }

    if (s_next == Sym("NIL")) {
      this.logger.info("Failure: Action " + a + " not applicable to " + s)
      s_next = s
    } else {
      this.logger.info("Success: " + s + " x " + a + " -> " + s_next)
    }
    s_next
  }

}
