package org.aiddl.example.learning_agent_scala.function

import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.{Bool, CollectionTerm, KeyVal, ListTerm, SetTerm, Sym, Term, Tuple, Num}
import org.aiddl.common.scala.planning.PlanningTerm
import org.aiddl.core.scala.util.ComboIterator

import scala.util.Random

object RandomSimCreator extends Function with Verbose {

  private val TransKey = Sym("state-transitions")
  private val ActionsKey = Sym("actions")
  override def apply(x: Term): Term = {
    val r = new Random
    val n: Int = x.intoInt

    var actions: Set[Term] = Set.empty
    var choices: List[List[Term]] = Nil

    for (i <- 0 until n) {
      val stateChoice: List[Term] = List(Bool(true), Bool(false))
      choices = stateChoice :: choices
    }
    var state_transitions: List[Term] = Nil
    val stateCombos: ComboIterator[Term] = new ComboIterator[Term](choices)
    for (combo <- stateCombos) {
      var s_set: Set[Term] = Set.empty
      var i: Int = 1
      while (i <= combo.size) {
        s_set = s_set + KeyVal(
          Tuple(Sym("light"), Num(i)),
          Bool(combo(i-1).equals(Bool(true))))
        i += 1
      }
      val s = SetTerm(s_set)

      this.logger.info("State: " + s)
      this.logger.depth += 1
      var transitions_for_s: Set[Term] = Set.empty
      var j: Int = 0
      while (j < r.nextInt(3) + 1) {
        var effects_set: Set[Term] = Set.empty
        var s_next: Set[Term] = Set.empty
        val numChanges: Int = r.nextInt(2) + 1
        var used: Set[Int] = Set.empty
        while (used.size <= numChanges) {
          used = used + (r.nextInt(n) + 1)
        }
        for (i <- 1 to n) {
          val variable = Tuple(Sym("light"), Num(i))
          if (used.contains(i)) {
            effects_set = effects_set + KeyVal(variable, Bool(!s(variable).intoBool.v))
            s_next = s_next + KeyVal(variable, Bool(!s(variable).intoBool.v))
          } else {
            s_next = s_next + KeyVal(variable, s(variable))
          }
        }
        val effects = SetTerm(effects_set)
        if (effects_set.isEmpty || transitions_for_s.contains(effects)) {
          j -= 1
        } else {
          val actionName = Tuple(Sym("push"), Num(r.nextInt(n) + 1))
          actions = actions + actionName
          state_transitions = KeyVal(Tuple(actionName, s), SetTerm(s_next)) :: state_transitions
          this.logger.info("Action: " + actionName)
          this.logger.info("Next state: " + s_next)
          j += 1
        }
      }
      this.logger.depth -= 1
    }

    this.logger.info("Transitions: " + state_transitions.size)
    this.logger.info("Actions:" + actions.size)

    ListTerm(
      KeyVal(TransKey, ListTerm(state_transitions)),
      KeyVal(ActionsKey, SetTerm(actions + Sym("reset")))
    )
  }
}
