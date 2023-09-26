package org.aiddl.common.scala.planning

import org.aiddl.common.scala.math.linear_algebra.Matrix
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.state_variable.{ProblemCompiler, ReachableOperatorEnumerator}
import org.aiddl.common.scala.planning.state_variable.heuristic.{CausalGraphHeuristic, FastForwardHeuristic, SumCostHeuristic}
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.scalatest.funsuite.AnyFunSuite

class HeuristicSuite extends AnyFunSuite {
    val p01 = {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("aiddl-test/planning/state-variable/elevator/problem-01.aiddl")
        assert(c.typeCheckModule(m))
        ReachableOperatorEnumerator.groundProblem(c.getProcessedValueOrPanic(m, Sym("problem")))
    }

    val p02 = {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("aiddl-test/planning/state-variable/elevator/problem-02.aiddl")
        assert(c.typeCheckModule(m))
        ReachableOperatorEnumerator.groundProblem(c.getProcessedValueOrPanic(m, Sym("problem")))

    }

    val p03 = {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("aiddl-test/planning/state-variable/elevator/problem-03.aiddl")
        assert(c.typeCheckModule(m))
        ReachableOperatorEnumerator.groundProblem(c.getProcessedValueOrPanic(m, Sym("problem")))
    }

    val p04 = {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("aiddl-test/planning/state-variable/elevator/problem-04.aiddl")
        assert(c.typeCheckModule(m))
        ReachableOperatorEnumerator.groundProblem(c.getProcessedValueOrPanic(m, Sym("problem")))
    }

    val p05 = {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("aiddl-test/planning/misc/test-01.aiddl")
        //assert(c.typeCheckModule(m))
        ReachableOperatorEnumerator.groundProblem(c.getProcessedValueOrPanic(m, Sym("problem")))
    }

    val h_+ = new SumCostHeuristic
    val h_cg = new CausalGraphHeuristic
    val h_ff = new FastForwardHeuristic

    test("Sum Cost heuristic value test 01") {
        h_+.init(p01)
        assert(h_+(p01(InitialState)) == Num(6))
    }

    test("Sum Cost heuristic value test 02") {
        h_+.init(p02)
        assert(h_+(p02(InitialState)) == Num(3))
    }

    test("Sum Cost heuristic value test 03") {
        h_+.init(p03)
        assert(h_+(p03(InitialState)) == InfPos())
    }

    test("Sum Cost heuristic value test 04") {
        h_+.init(p04)
        assert(h_+(p04(InitialState)) == Num(0))
    }

    test("Causal Graph heuristic value test 01") {
        h_cg.init(p01)
        assert(h_cg(p01(InitialState)) == Num(8))
    }

    test("Causal Graph heuristic value test 02") {
        h_cg.init(p02)
        assert(h_cg(p02(InitialState)) == Num(4))
    }

    test("Causal Graph heuristic value test 03") {
        h_cg.init(p03)
        assert(h_cg(p03(InitialState)) == InfPos())
    }

    test("Causal Graph heuristic value test 04") {
        h_cg.init(p04)
        assert(h_cg(p04(InitialState)) == Num(0))
    }

    test("Causal Graph heuristic value test 05") {
        h_cg.init(p05)
        assert(h_cg(p05(InitialState)) == Num(1))
    }

    test("Fast Forward Graph heuristic value test 01") {
        h_ff.init(p01)
        assert(h_ff(p01(InitialState)) == Num(8))
    }

    test("Fast Forward Graph heuristic value test 02") {
        h_ff.init(p02)
        assert(h_ff(p02(InitialState)) == Num(5))
    }

    test("Fast Forward Graph heuristic value test 03") {
        h_ff.init(p03)
        assert(h_ff(p03(InitialState)) == InfPos())
    }

    test("Fast Forward Graph heuristic value test 04") {
        h_ff.init(p04)
        assert(h_ff(p04(InitialState)) == Num(0))
    }
}
