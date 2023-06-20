package org.aiddl.common.scala.planning

import org.aiddl.common.scala.math.graph.Graph2Dot
import org.aiddl.common.scala.math.graph.GraphType.Directed
import org.aiddl.common.scala.math.linear_algebra.Matrix
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.state_variable.heuristic.{FastForwardHeuristic, Heuristic}
import org.aiddl.common.scala.planning.state_variable.{ForwardSearchPlanIterator, ProblemCompiler, ReachableOperatorEnumerator, UnboundEffectGrounder}
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.logger.Logger
import org.scalatest.funsuite.AnyFunSuite

import java.util.logging.Level

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

    val forwardPlanner = ForwardSearchPlanIterator()

    test("Heuristic plan search test 01") {
        forwardPlanner.init(p01)

        val plan = forwardPlanner.search
        assert(plan match
            case None => false
            case Some(list) => list.length == 6
        )
    }

    test("Multiple heuristics test 01") {
        val h_ff = new FastForwardHeuristic
        val h_cg = new FastForwardHeuristic

        val forwardPlanner = new ForwardSearchPlanIterator(List((h_cg, Num(1)), (h_ff, Num(0.8))))
        forwardPlanner.addHeuristic(h_ff, Num(1))
        forwardPlanner.init(p01)
        val plan = forwardPlanner.search

        println(Logger.prettyPrint(forwardPlanner.graph, 1))

        forwardPlanner.searchGraph2File("search.dot")

        assert(plan match
            case None => false
            case Some(list) => list.length == 6
        )
    }

    test("Operator grounding provides right number of operators (1)") {
        assert(p01(Sym("operators")).asCol.size == 40)
    }

    test("Operator grounding provides right number of operators (2)") {
        assert(p02(Sym("operators")).asCol.size == 6)
    }

    test("Operator grounding provides right number of operators (3)") {
        p03(Sym("operators")).asCol.foreach(println)
        assert(p03(Sym("operators")).asCol.size == 0)
    }

    test("Operator grounding provides right number of operators (4)") {
        //p02(Sym("operators")).asCol.foreach(println)
        assert(p04(Sym("operators")).asCol.size == 0)
    }

    test("Grounding unbound effects") {
        val problem = {
            val c = new Container()
            val parser = new Parser(c)
            parser.str("{" +
              "domains:[type:[a b c d]] " +
              "operators:{" +
              "  [name:(a ?x) signature:[?x:type] preconditions:{} effects:{(effect:?x)}]}" +
              "}")
        }

        //println(problem)

        val ueGrounder = new UnboundEffectGrounder
        val problemAfter = ueGrounder(problem)

        //println(problemAfter)

        assert(problemAfter(Sym("operators")).asCol.size == 4)
    }
}
