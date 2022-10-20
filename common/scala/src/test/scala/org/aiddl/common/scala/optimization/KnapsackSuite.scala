package org.aiddl.common.scala.optimization

import org.aiddl.common.scala.Common
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.constraint.CspSolver
import org.aiddl.common.scala.optimization.combinatorial.BranchAndBound
import org.aiddl.common.scala.optimization.combinatorial.knapsack.Knapsack.{Items, Value, Weight}
import org.aiddl.common.scala.optimization.combinatorial.knapsack.{Knapsack, KnapsackGenerator, KnapsackToCombinatorialOptimizationConverter}
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.function.DefaultFunctionUri.EVAL
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.{Logger, StopWatch}
import org.scalatest.funsuite.AnyFunSuite


class KnapsackSuite extends AnyFunSuite {
  val c = new Container()
  val parser = new Parser(c)
  val m1 = parser.parseFile("../test/optimization/combinatorial/knapsack/knapsack-01.aiddl")
  val m2 = parser.parseFile("../test/optimization/combinatorial/knapsack/knapsack-02.aiddl")
  val m3 = parser.parseFile("../test/optimization/combinatorial/knapsack/knapsack-03.aiddl")
  val m4 = parser.parseFile("../test/optimization/combinatorial/knapsack/knapsack-04.aiddl")
  val m5 = parser.parseFile("../test/optimization/combinatorial/knapsack/knapsack-05.aiddl")

  val converter = new KnapsackToCombinatorialOptimizationConverter

  test("Knapsack - Problem 01 - Branch and bound without propagation") {
    val p01 = c.getProcessedValueOrPanic(m1, Sym("problem"))
    val coSolver = new BranchAndBound()
    coSolver.init(p01)
    val aco = coSolver.optimal
    assert(coSolver.best == Num(11))
  }

  test("Knapsack - Problem 02 - Branch and bound with propagation") {
    val p02 = c.getProcessedValueOrPanic(m2, Sym("problem"))
    val coSolver = new BranchAndBound()
    val cop = converter(p02)

    coSolver.usePropagation = true
    coSolver.setRemainingCostFunction(Knapsack.remainingCostEstGeneratorFillWithBest(p02))
    coSolver.init(cop)
    coSolver.optimal
    assert(coSolver.best == Num(11))
  }

  test("Knapsack - Problem 03") {
    val problem = c.getProcessedValueOrPanic(m3, Sym("problem"))
    object coSolver extends BranchAndBound {
      //setRemainingCostFunction(Knapsack.remainingCostEstGeneratorFillWithBest(problem))
      //setRemainingCostFunction(Knapsack.remainingCostEstGeneratorFillWithBestLeft(problem))
      setRemainingCostFunction(Knapsack.remainingCostEstGeneratorFillGreedy(problem))
      staticVariableOrdering = Knapsack.variableOrderingGenerator(problem)
      staticValueOrdering = Knapsack.valueOrdering
      usePropagation = false
    }
    coSolver.init(converter(problem))
    val aco = coSolver.optimal
    assert(coSolver.best == Num(355))
  }

  test("Knapsack - Problem 04") {
    val problem = c.getProcessedValueOrPanic(m4, Sym("problem"))
    object coSolver extends BranchAndBound {
      //setRemainingCostFunction(Knapsack.remainingCostEstGeneratorFillWithBest(problem))
      //setRemainingCostFunction(Knapsack.remainingCostEstGeneratorFillWithBestLeft(problem))
      setRemainingCostFunction(Knapsack.remainingCostEstGeneratorFillGreedy(problem))
      staticVariableOrdering = Knapsack.variableOrderingGenerator(problem)
      staticValueOrdering = Knapsack.valueOrdering
      usePropagation = false
    }

    coSolver.init(converter(problem))
    val aco = coSolver.optimal
    assert(coSolver.best == Num(259))
  }

  /*
  test("Knapsack - Problem 05") {
    val problem = c.getProcessedValueOrPanic(m5, Sym("problem"))
    object coSolver extends BranchAndBound {
      setRemainingCostFunction(Knapsack.remainingCostEstGeneratorFillGreedy(problem))
      staticVariableOrdering = Knapsack.variableOrderingGenerator(problem)
      staticValueOrdering = Knapsack.valueOrdering
      usePropagation = false
      override def solutionFoundHook: Unit = {
        println(s"$best is new best cost with: $solution")
      }
    }

    coSolver.init(converter(problem))
    val aco = coSolver.optimal
    println(aco)

    println(coSolver.cConsistentNodes)
    println(coSolver.cDeadEnd)
    println(StopWatch.summary)
  }*/

  test("Knapsack - Generate Problem") {
    val generator = new KnapsackGenerator

    val problem = generator(parser.str("(" +
      "capacity:200 " +
      "per-item-limit:3 " +
      "items:10 " +
      "weight:(1 30) " +
      "value:(1 30)" +
      ")"))

    assert(problem(Items).length == 10)
  }
}