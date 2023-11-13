package org.aiddl.common.scala.optimization

import org.aiddl.common.scala.Common
import org.aiddl.common.scala.math.graph.{AdjacencyListGraph, Graph2Dot}
import org.aiddl.common.scala.math.graph.GraphType.Undirected
import org.aiddl.common.scala.optimization.combinatorial.tsp.{MinRemainder, PathExpander, TspGenerator, TspGreedyLocalSearch, TspRandomSolutionGenerator, TspSolver, TspUtils}
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.StopWatch
import org.aiddl.core.scala.util.logger.Logger
import org.scalatest.funsuite.AnyFunSuite

class TspSuite extends AnyFunSuite {
    test("TSP heuristic test") {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("aiddl-test/optimization/combinatorial/traveling-salesperson-problem/tsp-n03-01.aiddl")
        assert(c.typeCheckModule(m))
        val p = c.getProcessedValueOrPanic(m, Sym("problem"))
        val tspMinRemainder = new MinRemainder
        tspMinRemainder.init(p)
        assert( tspMinRemainder(ListTerm.empty) == Num(490) )
    }

    test("TSP test (n=3)") {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("aiddl-test/optimization/combinatorial/traveling-salesperson-problem/tsp-n03-01.aiddl")
        assert(c.typeCheckModule(m))
        val p = c.getProcessedValueOrPanic(m, Sym("problem"))
        val expander = new PathExpander
        expander.init(p)
        assert(expander(ListTerm.empty: Term) == ListTerm(Sym("n1")))
        assert(expander(ListTerm(Sym("n1")): Term).asSet == SetTerm(Sym("n2"), Sym("n3")))
        assert(expander(ListTerm(Sym("n2"), Sym("n3"), Sym("n1")): Term) == Common.NIL)
        val tspSolver = new TspSolver
        tspSolver.init(p)
        val sol = tspSolver.optimal
        assert( sol.get != Common.NIL )
        assert( tspSolver.best == Num(770) )
    }

    test("TSP test (n=4)") {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("aiddl-test/optimization/combinatorial/traveling-salesperson-problem/tsp-n04-01.aiddl")
        assert(c.typeCheckModule(m))
        val p = c.getProcessedValueOrPanic(m, Sym("problem"))

        val tspSolver = new TspSolver
        tspSolver.init(p)
        val sol = tspSolver.optimal

        assert( sol.get != Common.NIL )
        assert( tspSolver.best == Num(998) )
    }

    test("TSP test (n=5)") {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("aiddl-test/optimization/combinatorial/traveling-salesperson-problem/tsp-n05-01.aiddl")
        assert(c.typeCheckModule(m))
        val p = c.getProcessedValueOrPanic(m, Sym("problem"))

        val tspSolver = new TspSolver
        tspSolver.init(p)
        val sol = tspSolver.optimal
        assert( sol.get != Common.NIL )
        assert( tspSolver.best == Num(998) )
    }

    test("TSP Generator") {
        val tspGen = new TspGenerator
        val p = tspGen(5, 1000, 1000)
        assert(p(Sym("V")).length == 5)
    }

    test("Testing HillClimbing on TSP problem") {
        for (i <- 0 until 5) {
            val tspGen = new TspGenerator
            val p = tspGen(5, 1000, 1000)
            val graph = new AdjacencyListGraph(p)
            val solutionGenerator = new TspRandomSolutionGenerator(graph)
            val init = solutionGenerator()
            val hcSolver = new TspGreedyLocalSearch(init, graph) {
                numTries = 10
            }
            val answer = hcSolver.search

            val tspSolver = new TspSolver {
                traceFlag = true
            }
            tspSolver.init(p)
            val noBoundAnswer = tspSolver.optimal
            tspSolver.searchGraph2File("tsp.dot")
            assert((hcSolver.valueFunction(answer) - tspSolver.best).abs >= Num(0.0))
        }
    }
}
