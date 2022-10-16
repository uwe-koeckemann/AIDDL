package org.aiddl.common.scala.optimization

import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation._
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.parser.Parser

import org.aiddl.common.scala.learning.supervised.decision_tree.ID3
import org.aiddl.common.scala.learning.supervised.DataSplitter

import org.aiddl.core.scala.util.Logger

import org.aiddl.core.scala.representation.TermCollectionImplicits.term2ListTerm
import org.aiddl.common.scala.optimization.combinatorial.tsp.TspSolver
import org.aiddl.common.scala.optimization.combinatorial.tsp.MinRemainder
import org.aiddl.common.scala.Common
import org.aiddl.common.scala.optimization.combinatorial.tsp.TspGenerator

class TspSuite extends AnyFunSuite {
    test("TSP heuristic test") {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("../test/optimization/combinatorial/traveling-salesperson-problem/tsp-n03-01.aiddl")
        assert(c.typeCheckModule(m))
        val p = c.getProcessedValueOrPanic(m, Sym("problem"))

        val tspMinRemainder = new MinRemainder
        tspMinRemainder.init(p)

        assert( tspMinRemainder(ListTerm.empty) == Num(490) )
    }

    test("TSP test (n=3)") {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("../test/optimization/combinatorial/traveling-salesperson-problem/tsp-n03-01.aiddl")
        assert(c.typeCheckModule(m))
        val p = c.getProcessedValueOrPanic(m, Sym("problem"))

        val tspSolver = new TspSolver
        tspSolver.init(p)
        val sol = tspSolver.optimal

        assert( sol.get != Common.NIL )
        assert( tspSolver.best == Num(770) )
    }

    test("TSP test (n=4)") {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("../test/optimization/combinatorial/traveling-salesperson-problem/tsp-n04-01.aiddl")
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
        val m = parser.parseFile("../test/optimization/combinatorial/traveling-salesperson-problem/tsp-n05-01.aiddl")
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
        val p = tspGen(10, 1000, 1000)
        assert(p(Sym("V")).length == 10)
        //val tspSolver = new TspSolver
        //tspSolver.init(p)
        //val sol = tspSolver.optimal
        //assert( sol.get != Common.NIL )
    }
}
