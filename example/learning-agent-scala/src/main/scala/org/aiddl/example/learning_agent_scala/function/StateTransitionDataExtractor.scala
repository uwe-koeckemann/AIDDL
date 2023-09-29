package org.aiddl.example.learning_agent_scala.function

import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.{Bool, CollectionTerm, KeyVal, ListTerm, Num, SetTerm, Sym, Term, Tuple, Var}
import org.aiddl.common.scala.learning.LearningTerm
import org.aiddl.common.scala.planning.PlanningTerm

import java.util.logging.Level

class StateTransitionDataExtractor(val attributes: ListTerm) extends Function with Verbose {
  this.logConfig(level=Level.INFO)

  override def apply(x: Term): Term = {
    this.logger.info("Extracting Data...")
    val state = x(PlanningTerm.State).asSet
    val action = x(Sym("action"))
    val nextState = x(Sym("next-state")).asSet
    val data = x(LearningTerm.Data).asList

    this(state, action, nextState, data)
  }

  def apply(state: Term, action: Term, nextState: SetTerm, data: ListTerm): ListTerm = {
    var updatedData: ListTerm = ListTerm.empty
    var effects: Set[Term] = Set.empty
    for (p <- nextState) {
      effects += p
    }
    var newDataPoint: List[Term] = Nil

    var i = 0
    while (i < attributes.size - 2) {
      val atom = attributes(i).asTup
      newDataPoint = state(atom) :: newDataPoint
      i += 1
    }
    newDataPoint = action :: newDataPoint
    newDataPoint = SetTerm(effects) :: newDataPoint

    val newDataPointTerm = ListTerm(newDataPoint.reverse)
    updatedData = data.add(newDataPointTerm)

    if (updatedData.size != data.size) {
      this.logger.info("New data point: " + newDataPointTerm)
    }
    this.logger.depth += 1
    for (d <- updatedData) {
      this.logger.fine(d.toString)
    }
    this.logger.depth -= 1
    this.logger.info(s"|Data| = ${updatedData.size}")
    updatedData
  }

}
