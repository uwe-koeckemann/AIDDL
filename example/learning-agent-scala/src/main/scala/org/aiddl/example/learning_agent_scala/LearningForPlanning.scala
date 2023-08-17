package org.aiddl.example.learning_agent_scala

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
import org.aiddl.core.scala.util.logger.Logger
import org.aiddl.example.learning_agent_scala.function.{ActionExecutor, ActionSelector, OperatorCreator, PlanSuccessChecker, RandomSimCreator, SleepAndLog, StateTransitionDataExtractor}

import java.util.logging.Level

object LearningForPlanning extends Verbose {
  this.logConfig(level=Level.INFO)

  private val dataSplitter = new DataSplitter
  private val id3 = new ID3
  id3.includeAllLeafs = true
  private val forwardPlan = ForwardSearchPlanIterator()

  var iterationCount = 0
  def run = {
    val container = new Container
    val parser = new Parser(container)
    val moduleUri = parser.parseFile("./aiddl/learning-for-planning-domain.aiddl")

    // Load processed terms from AIDDL module
    val planningDomain = container.getProcessedValueOrPanic(moduleUri, Sym("PlanningProblem"))
    val numberOfLights = container.getProcessedValueOrPanic(moduleUri, Sym("NumLights"))
    val planAttributes = container.getProcessedValueOrPanic(moduleUri, Sym("PlanAttributes")).asList

    // Set planning related variables and values
    var state = planningDomain(InitialState).asSet
    var operators = planningDomain(Operators).asSet
    val goal = planningDomain(Goal).asSet
    var plan: Term = Common.NIL

    // Initially empty data set
    var data = ListTerm.empty

    // Create a random state-transition map (this is what our operators should capture)
    val simulation = RandomSimCreator(numberOfLights)
    val simStateTransitions = simulation(Sym("state-transitions")).asCol
    val simActions = simulation(Sym("actions")).asList

    // Initiate functions used below
    val actionSelector = new ActionSelector(simActions)
    val actionExecutor = new ActionExecutor(simStateTransitions, state)
    val dataExtractor = new StateTransitionDataExtractor(planAttributes.asList)
    val operatorCreator = new OperatorCreator(planAttributes)

    while (true) {
      this.iterationCount += 1
      SleepAndLog(100, this.iterationCount)
      val actionSelectorResult = actionSelector(plan)
      val selectedAction = actionSelectorResult(Sym("selected-action"))
      val planTail = actionSelectorResult(Sym("plan-tail"))
      plan = planTail
      this.logger.info(s"Selected action: $selectedAction (plan tail: $plan)")

      val stateNext = actionExecutor(selectedAction, state).asSet

      if (selectedAction != Sym("reset")) {
        // Add most recent transition to data set
        data = dataExtractor(state, selectedAction, stateNext, data)
        // Assemble learning task to predict effects from current state and action
        val learningTask = Tuple(
          KeyVal(Attributes, planAttributes),
          KeyVal(Label, Sym("Effects")),
          KeyVal(Data, data))
        // Split learning task into data matrix x (composed of non label columns) and prediction column y (the label column)
        val (x, y) = dataSplitter(learningTask) match {
          case Tuple(x, y) => (x.asList, y.asList)
          case _ => ???
        }
        // Use ID3 algorithm to learn a decision tree model
        val decisionTree = id3.fit(x, y)
        // Convert decision tree to a set of operators
        operators = operatorCreator(decisionTree).asSet
      }
      state = stateNext
      if (plan == Common.NIL) {
        // If there is not active plan, assemble and solve problem with current operator set
        forwardPlan.init(
          Tuple(
            KeyVal(InitialState, state),
            KeyVal(Operators, operators),
            KeyVal(Goal, goal)
          ))
        plan = forwardPlan.search match
          case Some(plan) => ListTerm(plan)
          case None => Common.NIL
        this.logger.info(s"Planning result: $plan")
      }
      if (plan == ListTerm.empty) {
        // If all actions in plan have been executed, check if we reached our goal
        if (state containsAll goal) {
          state = planningDomain(InitialState).asSet
          this.logger.info(">>> GOAL REACHED BY PLAN!")
          Thread.sleep(5000)
        } else {
          this.logger.info(">>> GOAL FAILED...")
        }
        plan = Common.NIL
      }
      val planSize = if plan == Sym("NIL") then 0 else plan.asCol.size
      this.logger.info(s"|Data| = ${data.size} |O| = ${operators.size} |pi| = $planSize")
    }
  }
}

