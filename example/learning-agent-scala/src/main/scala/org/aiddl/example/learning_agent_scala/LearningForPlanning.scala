package my.org.aiddl_sbt_project

import org.aiddl.common.scala.Common
import org.aiddl.common.scala.learning.supervised.decision_tree.ID3
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.learning.LearningTerm.*
import org.aiddl.common.scala.learning.supervised.DataSplitter
import org.aiddl.common.scala.planning.state_variable.ForwardSearchPlanIterator
import org.aiddl.common.scala.planning.state_variable.heuristic.FastForwardHeuristic
import org.aiddl.core.scala.representation.{Bool, CollectionTerm, KeyVal, ListTerm, Num, SetTerm, Sym, Term, Tuple, Var}
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.parser.Parser
import org.aiddl.example.learning_agent_scala.function.{ActionExecutor, ActionSelector, OperatorCreator, PlanSuccessChecker, RandomSimCreator, SleepAndLog, StateTransitionDataExtractor}

object LearningForPlanning extends Verbose {
  var iterationCount = 0
  def run = {
    val container = new Container
    val parser = new Parser(container)

    val moduleUri = parser.parseFile("./aiddl/learning-for-planning-domain.aiddl")

    val planningDomain = container.getProcessedValueOrPanic(moduleUri, Sym("Pi"))
    val numberOfLights = container.getProcessedValueOrPanic(moduleUri, Sym("NumLights"))
    val planAttributes = container.getProcessedValueOrPanic(moduleUri, Sym("PlanAttributes")).asList

    var state = planningDomain(InitialState)
    var operators = planningDomain(Operators).asSet
    val goal = planningDomain(Goal).asSet
    var plan: Term = Common.NIL

    var data = ListTerm.empty

    val simulation = RandomSimCreator(numberOfLights)
    val simStateTransitions = simulation(Sym("state-transitions")).asCol
    val simActions = simulation(Sym("actions")).asList

    val actionSelector = new ActionSelector(simActions)
    val actionExecutor = new ActionExecutor(simStateTransitions)
    val dataExtractor = new StateTransitionDataExtractor(planAttributes.asList)
    val id3 = new ID3
    val dataSplitter = new DataSplitter
    val forwardPlan = ForwardSearchPlanIterator()

    while (true) {
      this.iterationCount += 1
      SleepAndLog(1000, this.iterationCount)

      val actionSelectorResult = actionSelector(plan)
      val selectedAction = actionSelectorResult(Sym("selected-action"))
      val planTail = actionSelectorResult(Sym("plan-tail"))
      plan = planTail
      val stateNext = actionExecutor(selectedAction, state)

      if (selectedAction != Tuple(Sym("reset"))) {
        data = dataExtractor(state, selectedAction, stateNext, data)
        val learningTask = Tuple(
          KeyVal(Attributes, planAttributes),
          KeyVal(Label, Sym("Effects")),
          KeyVal(Data, data)
        )

        val (x, y) = dataSplitter(learningTask) match {
          case Tuple(x, y) => (x.asList, y.asList)
          case _ => ???
        }
        val decisionTree = id3.fit(x, y)
        operators = OperatorCreator(decisionTree).asSet
      }
      state = stateNext
      if (plan == Common.NIL) {
        forwardPlan.init(
          Tuple(
            KeyVal(InitialState, state),
            KeyVal(Operators, operators),
            KeyVal(Goal, goal)
          ))
        plan = forwardPlan.search match
          case Some(plan) => ListTerm(plan)
          case None => Common.NIL
      }
      if (plan == ListTerm.empty) {
        if (state.asSet containsAll goal) {
          state = planningDomain(InitialState)
          plan = Common.NIL
          this.logger.info(">>> GOAL REACHED BY PLAN!")
        } else {
          this.logger.info(">>> GOAL FAILED...")
        }
      }
      val planSize = if plan == Sym("NIL") then 0 else plan.asCol.size
      this.logger.info(s"|Data| = ${data.size} |O| = ${operators.size} |pi| = $planSize")
    }
  }
}

