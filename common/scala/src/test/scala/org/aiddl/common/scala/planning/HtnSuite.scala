package org.aiddl.common.scala.planning

import org.aiddl.common.scala.Common
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.linear_algebra.Matrix
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.state_variable.{ProblemCompiler, ReachableOperatorEnumerator}
import org.aiddl.common.scala.planning.state_variable.heuristic.{CausalGraphHeuristic, FastForwardHeuristic, SumCostHeuristic}
import org.aiddl.common.scala.planning.task_network.TotalOrderForwardDecomposition
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.function.DefaultFunctionUri.EVAL
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.scalatest.funsuite.AnyFunSuite

class HtnSuite extends AnyFunSuite {

  val p01 = {
    val c = new Container()
    Function.loadDefaultFunctions(c)
    val eval = c.getFunctionOrPanic(EVAL)
    val parser = new Parser(c)
    val m = parser.parseFile("../test/planning/task-network/dock-worker-robot/problem-01.aiddl")
    assert(c.typeCheckModule(m))
    eval(c.getProcessedValueOrPanic(m, Sym("problem")))
  }

  val p02 = {
    val c = new Container()
    val parser = new Parser(c)
    Function.loadDefaultFunctions(c)
    val eval = c.getFunctionOrPanic(EVAL)
    val m = parser.parseFile("../test/planning/task-network/dock-worker-robot/problem-02.aiddl")
    assert(c.typeCheckModule(m))
    eval(c.getProcessedValueOrPanic(m, Sym("problem")))
  }

  val p03 = {
    val c = new Container()
    val parser = new Parser(c)
    Function.loadDefaultFunctions(c)
    val eval = c.getFunctionOrPanic(EVAL)
    val m = parser.parseFile("../test/planning/task-network/dock-worker-robot/problem-03.aiddl")
    assert(c.typeCheckModule(m))
    eval(c.getProcessedValueOrPanic(m, Sym("problem")))
  }

  val toDecomp = new TotalOrderForwardDecomposition

  test("Total-order Decomposition - Problem 01") {
    toDecomp.init(p01)
    var s = toDecomp(p01)
    assert( s.length == 4 )
  }

  test("Total-order Decomposition - Problem 02") {
    toDecomp.init(p02)
    var s = toDecomp(p02)
    assert( s != NIL )
  }

  test("Total-order Decomposition - Problem 03") {
    toDecomp.init(p03)
    var s = toDecomp(p03)
    assert( s == NIL )
  }
}