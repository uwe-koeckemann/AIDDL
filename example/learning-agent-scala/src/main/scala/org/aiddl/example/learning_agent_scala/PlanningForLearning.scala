package org.aiddl.example.learning_agent_scala

import org.aiddl.common.scala.Common
import org.aiddl.common.scala.learning.LearningTerm.*
import org.aiddl.common.scala.learning.supervised.DataSplitter
import org.aiddl.common.scala.learning.supervised.decision_tree.ID3
import org.aiddl.common.scala.learning.supervised.score.Accuracy
import org.aiddl.common.scala.learning.testing.FoldDivider
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.state_variable.ForwardSearchPlanIterator
import org.aiddl.core.scala.representation.{KeyVal, ListTerm, Num, SetTerm, Sym, Term, Tuple}
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.parser.Parser
import org.aiddl.example.learning_agent_scala.function.{ActionSelector, DataGoalGenerator, HiddenModelCreator, OperatorExecutor, SampleDataExtractor, SleepAndLog}

import java.util.logging.Level

object PlanningForLearning extends Verbose {
  // Configuration for logging and sleep durations
  this.logConfig(level=Level.INFO)
  private val iterationSleepMs = 1000

  private var iterationCount = 0

  private val container = new Container
  private val parser = new Parser(container)
  private val moduleUri = parser.parseFile("./aiddl/planning-for-learning-domain.aiddl")

  private var state = container.getProcessedValueOrPanic(moduleUri, Sym("state")).asSet
  private val operators = container.getProcessedValueOrPanic(moduleUri, Sym("operators")).asCol
  private val locations = container.getProcessedValueOrPanic(moduleUri, Sym("d_loc")).asCol
  private val configurations = container.getProcessedValueOrPanic(moduleUri, Sym("d_config")).asCol
  private val labels = container.getProcessedValueOrPanic(moduleUri, Sym("d_label")).asList
  private val attributes = container.getProcessedValueOrPanic(moduleUri, Sym("attributes")).asList

  private val hiddenModelCreator = new HiddenModelCreator
  private val hiddenModel = hiddenModelCreator(locations, configurations, labels)

  private val forwardPlan = ForwardSearchPlanIterator()
  private val dataFoldDivider = new FoldDivider
  private val actionSelector = new ActionSelector(ListTerm.empty)
  private val operatorExecutor = new OperatorExecutor(operators)
  private val dataExtractor = new SampleDataExtractor(hiddenModel)
  private val dataSplitter = new DataSplitter
  private val id3 = new ID3
  private val accuracy = new Accuracy

  private var data: ListTerm = ListTerm.empty
  private var acc: Num = Num(0.0)
  private var goal: Term = _
  private var currentPlan: Term = Common.NIL

  def run(): Unit = {
    println(hiddenModel)

    while ( data.length < 100 || acc < Num(0.99) ) {
      sleepAndLog()

      this.selectDataGoal()
      this.plan()
      this.execute()
      this.extractDataAndResetState()
      if ( data.size > 10 ) {
        this.learnAndEvaluate()
      }
    }
  }

  def sleepAndLog(): Unit = {
    SleepAndLog(this.iterationCount, this.iterationSleepMs, this.logger)
    logger.info(s"|Data| = ${data.size} ACC = $acc")
    this.iterationCount += 1
  }

  private def selectDataGoal(): Unit = {
    this.goal = DataGoalGenerator(locations, configurations, data)
    this.logger.info(s"Data goal: $goal")
  }

  private def plan(): Unit = {
    this.currentPlan = {
      forwardPlan.init(
        Tuple(KeyVal(InitialState, state), KeyVal(Operators, operators), KeyVal(Goal, goal)))
      //forwardPlan.groundOperators.asList.foreach(println)
      forwardPlan.search match
        case Some(plan) => ListTerm(plan)
        case None => Common.NIL
    }
  }

  private def execute(): Unit = {
    while (currentPlan != Common.NIL && currentPlan.asList.nonEmpty) {
      val (selectedAction, planTail) = actionSelector.select(currentPlan)
      state = operatorExecutor(state, selectedAction)
      currentPlan = planTail
    }
  }

  private def extractDataAndResetState(): Unit = {
    val (newData, nextState) = dataExtractor(state, data)
    data = newData
    state = nextState
  }

  private def learnAndEvaluate(): Num = {
    this.logger.info("Learning")
    var allPredicted = ListTerm.empty
    var allTest = ListTerm.empty
    for (i <- 1 to 10) {
      val (test, train) = dataFoldDivider(i, 10, data)
      val (xTrain, yTrain) = dataSplitter(attributes, Sym("Label"), train)
      id3.fit(xTrain, yTrain)
      val (xTest, yTest) = dataSplitter(attributes, Sym("Label"), test)
      val yPred = id3.predict(xTest)
      allPredicted = allPredicted.addAll(yPred)
      allTest = allTest.addAll(yTest)
    }
    acc = accuracy.score(allPredicted, allTest)
    println(s"Overall accuracy: $acc")
    acc
  }
}