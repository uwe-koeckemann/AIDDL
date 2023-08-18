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
import org.aiddl.example.learning_agent_scala.function.{ActionExecutor, ActionSelector, OperatorCreator, PlanSuccessChecker, RandomSimCreator, StateTransitionDataExtractor}

import java.util.logging.Level

object LearningForPlanning extends Verbose {
  // Configuration for logging and sleep durations
  this.logConfig(level=Level.INFO)
  private val iterationSleepMs = 1
  private val successSleepMs = 1

  // Load an AIDDL file (aka module) into a container
  val container = new Container
  val parser = new Parser(container)
  private val moduleUri = parser.parseFile("./aiddl/learning-for-planning-domain.aiddl")

  // Load (processed) terms from AIDDL module
  private val planningDomain = container.getProcessedValueOrPanic(moduleUri, Sym("PlanningProblem"))
  private val numberOfLights = container.getProcessedValueOrPanic(moduleUri, Sym("NumLights"))
  private val planAttributes = container.getProcessedValueOrPanic(moduleUri, Sym("PlanAttributes")).asList

  // Constant symbol used for reset action
  private val RESET_ACTION = Sym("reset")

  // Relevant data
  private var data: ListTerm = ListTerm.empty
  private var state: SetTerm = planningDomain(InitialState).asSet
  private var stateNext: SetTerm = planningDomain(InitialState).asSet
  private var operators: SetTerm = planningDomain(Operators).asSet
  private val goal: SetTerm = planningDomain(Goal).asSet
  private var selectedAction: Term = Common.NIL
  private var currentPlan: Term = Common.NIL

  // Create a random state-transitions (used to execute actions)
  private val simulation = RandomSimCreator(numberOfLights)
  private val simStateTransitions = simulation(Sym("state-transitions")).asCol
  private val simActions = simulation(Sym("actions")).asList

  // Initiate AI and execution functions
  private val actionSelector = new ActionSelector(simActions)
  private val actionExecutor = new ActionExecutor(simStateTransitions, state)
  private val dataExtractor = new StateTransitionDataExtractor(planAttributes.asList)
  private val operatorCreator = new OperatorCreator(planAttributes)
  private val dataSplitter = new DataSplitter
  private val id3 = new ID3
  id3.includeAllLeafs = true
  private val forwardPlan = ForwardSearchPlanIterator()

  // Counters used to decide when to stop
  private var iterationCount = 1
  private var successCount = 0
  private var failCount = 0

  def run(): Unit = {
    while ( limitNotReached ) {
      sleepAndLog()
      act()
      if (selectedAction != RESET_ACTION) {
        learn()
      }
      this.state = stateNext
      if (currentPlan == Common.NIL) {
        plan()
      }
      if (currentPlan == ListTerm.empty) {
        checkSuccess()
        currentPlan = Common.NIL
      }
    }
    this.logger.info(s"Iterations: $iterationCount Success: $successCount Failure: $failCount")
  }

  private def limitNotReached: Boolean =
    iterationCount < 1000
      && successCount < 100
      && failCount < 100

  /**
   * Short sleep and logging
   */
  private def sleepAndLog(): Unit = {
    Thread.sleep(iterationSleepMs)
    logger.info("================================================================================")
    logger.info(s"= Iteration $iterationCount")
    logger.info("================================================================================")
    val planSize = if currentPlan == Sym("NIL") then 0 else currentPlan.asCol.size
    logger.info(s"|Data| = ${data.size} |O| = ${operators.size} |pi| = $planSize")
    iterationCount += 1
  }

  /**
   * Determine next action, execute it, and record resulting state state
   */
  private def act(): Unit = {
    val (action, planTail) = actionSelector.select(currentPlan)
    currentPlan = planTail
    this.selectedAction = action
    this.stateNext = actionExecutor(selectedAction, state).asSet
    this.logger.info(s"Selected action: $selectedAction (plan tail: $currentPlan)")
  }

  /**
   * Extract data, learn decision tree, and extract new operators from tree
   */
  private def learn(): Unit = {
    data = dataExtractor(state, selectedAction, this.stateNext, data)
    val (x, y) = dataSplitter(planAttributes, Sym("Effects"), data)
    val decisionTree = id3.fit(x, y)
    this.operators = operatorCreator(decisionTree)
  }

  /**
   * If there is no plan, attempt to find one
   */
  private def plan(): Unit = {
    forwardPlan.init(Tuple(KeyVal(InitialState, state), KeyVal(Operators, operators), KeyVal(Goal, goal)))
    this.currentPlan = forwardPlan.search match
      case Some(plan) => ListTerm(plan)
      case None => Common.NIL
  }

  /**
   * Check if plan execution was successful
   */
  private def checkSuccess(): Unit = {
    if (state containsAll goal) {
      state = planningDomain(InitialState).asSet
      successCount += 1
      this.logger.info(">>> GOAL REACHED BY PLAN!")
      Thread.sleep(successSleepMs)
    } else {
      failCount += 1
      this.logger.info(">>> GOAL FAILED...")
    }
  }
}

