package org.aiddl.example.learning_agent_scala.function

import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.{Bool, CollectionTerm, KeyVal, ListTerm, SetTerm, Sym, Term, Tuple, Num, Var}
import org.aiddl.common.scala.learning.LearningTerm
import org.aiddl.common.scala.planning.PlanningTerm

class StateTransitionDataExtractor(val attributes: ListTerm) extends Function with Verbose {

  override def apply(x: Term): Term = {
    this.logger.info("Extracting Data...")
    val state = x(PlanningTerm.State).asSet
    val action = x(Sym("action"))
    val nextState = x(Sym("next-state")).asSet
    val data = x(LearningTerm.Data).asList

    this(state, action, nextState, data)
  }

  def apply(state: Term, action: Term, nextState: SetTerm, data: ListTerm): ListTerm = {
    var updatedData: SetTerm = SetTerm.empty
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

    val newDataPointTerm = ListTerm(newDataPoint)
    updatedData = currentData.add(newDataPointTerm)

    if (updatedData.size != currentData.size) {
      this.logger.info("New data point: " + newDataPoint)
    } else {
      this.logger.info("Nothing new: " + newDataPoint)
    }
    this.logger.depth += 1
    for (d <- updatedData) {
      this.logger.info(d.toString)
    }
    this.logger.depth -= 1
    this.logger.info(s"|Data| = ${updatedData.size}")
    updatedData
  }

}
