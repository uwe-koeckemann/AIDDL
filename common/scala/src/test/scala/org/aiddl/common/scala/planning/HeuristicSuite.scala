package org.aiddl.common.scala.planning

import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation.Var
import org.aiddl.core.scala.representation.Num
import org.aiddl.core.scala.parser.Parser
import org.aiddl.common.scala.math.linear_algebra.Matrix
import org.aiddl.common.scala.planning.state_variable.ReachableOperatorEnumerator
import org.aiddl.common.scala.planning.state_variable.ProblemCompiler
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.state_variable.heuristic.{CausalGraphHeuristic, FastForwardHeuristic, SumCostHeuristic}
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.InfPos

class HeuristicSuite extends AnyFunSuite {
    val p01 = {
        val c = new Container()
        val m = Parser.parseInto("../test/planning/state-variable/elevator/problem-01.aiddl", c)
        assert(c.typeCheckModule(m))
        c.resolve(c.getEntry(m, Sym("problem")).get.v)
    }

    val p02 = {
        val c = new Container()
        val m = Parser.parseInto("../test/planning/state-variable/elevator/problem-02.aiddl", c)
        assert(c.typeCheckModule(m))
        c.resolve(c.getEntry(m, Sym("problem")).get.v)
    }

    val p03 = {
        val c = new Container()
        val m = Parser.parseInto("../test/planning/state-variable/elevator/problem-03.aiddl", c)
        assert(c.typeCheckModule(m))
        c.resolve(c.getEntry(m, Sym("problem")).get.v)
    }

    val p04 = {
        val c = new Container()
        val m = Parser.parseInto("../test/planning/state-variable/elevator/problem-04.aiddl", c)
        assert(c.typeCheckModule(m))
        c.resolve(c.getEntry(m, Sym("problem")).get.v)
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
