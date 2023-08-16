package org.aiddl.example.learning_agent_scala.function

import org.aiddl.common.scala.planning.PlanningTerm
import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.{CollectionTerm, Sym, Term, Tuple}

class ActionExecutor(val transitions: CollectionTerm) extends Function with Verbose {
  override def apply(x: Term): Term = {
    val action = x(Sym("action"))
    val state = x(PlanningTerm.State)
    this(state, action)
  }

  def apply(action: Term, state: Term): Term = {
    transitions.get(Tuple(action, state)) match
      case Some(stateNext) => {
        logger.info(s"Success: $state x $action -> $stateNext")
        stateNext
      }
      case None => {
        logger.info(s"Failure: Action $action not applicable to $state")
        state
      }
  }
}
