package org.aiddl.common.scala.planning

import org.aiddl.common.scala.math.linear_algebra.Matrix
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.state_variable.{ForwardSearchPlanIterator, ProblemCompiler, ReachableOperatorEnumerator}
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.scalatest.funsuite.AnyFunSuite

class PlannerSuite extends AnyFunSuite {

    val p01 = {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("../test/planning/state-variable/elevator/problem-01.aiddl")
        assert(c.typeCheckModule(m))
        ReachableOperatorEnumerator.groundProblem(c.resolve(c.getEntry(m, Sym("problem")).get.value))
    }

    val p02 = {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("../test/planning/state-variable/elevator/problem-02.aiddl")
        assert(c.typeCheckModule(m))
        ReachableOperatorEnumerator.groundProblem(c.resolve(c.getEntry(m, Sym("problem")).get.value))
    }

    val p03 = {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("../test/planning/state-variable/elevator/problem-03.aiddl")
        assert(c.typeCheckModule(m))
        ReachableOperatorEnumerator.groundProblem(c.resolve(c.getEntry(m, Sym("problem")).get.value))
    }

    val p04 = {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("../test/planning/state-variable/elevator/problem-03.aiddl")
        assert(c.typeCheckModule(m))
        ReachableOperatorEnumerator.groundProblem(c.resolve(c.getEntry(m, Sym("problem")).get.value))
    }

    val forwardPlanner = new ForwardSearchPlanIterator

    test("Sum Cost heuristic value test 01") {
        forwardPlanner.init(p01)
        val plan = forwardPlanner.search
        assert(plan match
            case None => false
            case Some(list) => list.length == 6
        )
    }
}