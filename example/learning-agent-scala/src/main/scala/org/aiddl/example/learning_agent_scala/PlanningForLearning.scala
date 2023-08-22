package org.aiddl.example.learning_agent_scala

import org.aiddl.common.scala.Common
import org.aiddl.common.scala.learning.LearningTerm.*
import org.aiddl.common.scala.learning.supervised.DataSplitter
import org.aiddl.common.scala.learning.supervised.decision_tree.ID3
import org.aiddl.common.scala.learning.supervised.score.Accuracy
import org.aiddl.common.scala.learning.testing.FoldDivider
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.state_variable.ForwardSearchPlanIterator
import org.aiddl.core.scala.representation.{KeyVal, ListTerm, Num, SetTerm, Sym, Tuple}
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.parser.Parser
import org.aiddl.example.learning_agent_scala.function.{ActionSelector, DataGoalGenerator, HiddenModelCreator, OperatorExecutor, SampleDataExtractor, SleepAndLog}

import java.util.logging.Level

object PlanningForLearning extends Verbose {
  // Configuration for logging and sleep durations
  this.logConfig(level=Level.INFO)
  private val iterationSleepMs = 1000

  var iterationCount = 0

  private var data: ListTerm = ListTerm.empty
  private var acc: Num = Num(0.0)

  def run(): Unit = {
    val container = new Container
    val parser = new Parser(container)

    val moduleUri = parser.parseFile("./aiddl/planning-for-learning-domain.aiddl")

    var state = container.getProcessedValueOrPanic(moduleUri, Sym("state")).asSet
    var operators = container.getProcessedValueOrPanic(moduleUri, Sym("operators")).asCol
    val locations = container.getProcessedValueOrPanic(moduleUri, Sym("d_loc")).asCol
    val configurations = container.getProcessedValueOrPanic(moduleUri, Sym("d_config")).asCol
    val labels = container.getProcessedValueOrPanic(moduleUri, Sym("d_label")).asList
    val attributes = container.getProcessedValueOrPanic(moduleUri, Sym("attributes")).asList

    val hiddenModelCreator = new HiddenModelCreator
    val hiddenModel = hiddenModelCreator(locations, configurations, labels)

    val forwardPlan = ForwardSearchPlanIterator()

    val dataFoldDivider = new FoldDivider

    val actionSelector = new ActionSelector(ListTerm.empty)
    val operatorExecutor = new OperatorExecutor(operators)
    val dataExtractor = new SampleDataExtractor(hiddenModel)
    val dataSplitter = new DataSplitter
    val id3 = new ID3
    val accuracy = new Accuracy



    println(hiddenModel)

    while ( data.length < 100 || acc < Num(0.99) ) {
      sleepAndLog()

      val goal = DataGoalGenerator(locations, configurations, data)
      this.logger.info(s"Data goal: $goal")
      //val goal = parser.str("{(sample slot-1):(data loc-1 c2)}")
      //val goal = parser.str("{(collected (data loc-1 c2)):true}")
      //forwardPlan.logConfig(level=Level.INFO)
      var plan = {
        forwardPlan.init(
          Tuple(KeyVal(InitialState, state), KeyVal(Operators, operators), KeyVal(Goal, goal)))
        //forwardPlan.groundOperators.asList.foreach(println)
        forwardPlan.search match
          case Some(plan) => ListTerm(plan)
          case None => Common.NIL
      }
      this.logger.info(s"State: $state")
      //forwardPlan.searchGraph2File("search.dot")
      this.logger.info(s"Plan: $plan")
      while ( plan != Common.NIL && !plan.asList.isEmpty ) {
        val (selectedAction, planTail) = actionSelector.select(plan)
        state = operatorExecutor(state, selectedAction)
        plan = planTail
      }
      val (newData, nextState) = dataExtractor(state, data)
      data = newData
      state = nextState
      if ( data.size > 10 ) {
        this.logger.info("Learning")
        var allPredicted = ListTerm.empty
        var allTest = ListTerm.empty
        for ( i <- 1 to 10 ) {
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
      }
    }

  }

  def sleepAndLog(): Unit = {
    SleepAndLog(this.iterationCount, this.iterationSleepMs, this.logger)
    logger.info(s"|Data| = ${data.size} ACC = ${acc}")
    this.iterationCount += 1
  }
}