package org.aiddl.example.learning_agent_scala.function

import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.{CollectionTerm, KeyVal, ListTerm, SetTerm, Sym, Term, Tuple, Bool}
import org.aiddl.common.scala.planning.PlanningTerm

object PlanSuccessChecker extends Function with Verbose {

  override def apply(x: Term): Term = {
    val goal = x(PlanningTerm.Goal).asSet
    val state = x(PlanningTerm.State).asSet
    Bool(this(goal, state))
  }

  def apply(state: SetTerm, goal: SetTerm): Boolean = {
    val success = state.containsAll(goal)

    if (success) {
      this.logger.info(">>>>>>>>>>>>>>>>>>>>>>>>>>>")
      this.logger.info(">>> GOAL REACHED BY PLAN!")
      this.logger.info(">>>>>>>>>>>>>>>>>>>>>>>>>>>")
      try {
        Thread.sleep(1000)
      } catch {
        case e1: InterruptedException =>
          e1.printStackTrace()
      }
    } else {
      this.logger.info(">>>>>>>>>>>>>>>>>>>>>>>>>>>")
      this.logger.info(">>> GOAL FAILED...")
      this.logger.info(">>>>>>>>>>>>>>>>>>>>>>>>>>>")
    }
    success
  }
}
