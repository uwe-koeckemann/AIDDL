package org.aiddl.common.scala.reasoning.temporal

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.temporal.{AllenInterval2Stp, StpSolver}
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.util.logger.Logger
import org.aiddl.core.scala.representation.*
import org.scalatest.funsuite.AnyFunSuite


class TemporalSuite extends AnyFunSuite {
    val fStp = new StpSolver
    val fAllen2Stp = new AllenInterval2Stp

    test("Stp solver satisfiable problems") {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("../test/reasoning/temporal/stp.aiddl")
        assert(c.typeCheckModule(m))
        val es = c.getMatchingEntries(m, Var(), Tuple(Sym("stp-consistent"), Var()))
        
        es.foreach( e => {
            val stp = e.value
            assert( fStp(stp) != NIL )
        })
    }

    test("Stp solver unsatisfiable problems") {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("../test/reasoning/temporal/stp.aiddl")
        assert(c.typeCheckModule(m))
        val es = c.getMatchingEntries(m, Var(), Tuple(Sym("stp-inconsistent"), Var()))
        
        es.foreach( e => {
            val stp = e.value
            assert( fStp(stp) == NIL )
        })
    }

    test("Allen cosntraints satisfiable problems") {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("../test/reasoning/temporal/allen-interval-constraints.aiddl")
        var es = c.getMatchingEntries(m, Var(), Tuple(Sym("consistent"), Var()))

        es.foreach( e => {
            val acs = e.value
            assert( fStp(fAllen2Stp(acs)) != NIL )
        })
    }

    test("Allen cosntraints unsatisfiable problems") {
        val c = new Container()
        val parser = new Parser(c)
        val m = parser.parseFile("../test/reasoning/temporal/allen-interval-constraints.aiddl")
        val es = c.getMatchingEntries(m, Var(), Tuple(Sym("inconsistent"), Var()))

        es.foreach( e => {
            val acs = e.value
            val stp = fAllen2Stp(acs)
            val answer = fStp(stp)
            assert( answer == NIL )
        })
    }

    test("Minimal Inconsistent Simple Temporal Problem") {
        val c = new Container()
        val parser = new Parser(c)

        val stp = parser.str("({ x1 x2 x3 x4 x5 } { (x1 x2 5 5) (x2 x3 5 5) (x3 x1 5 5) (x3 x4 5 5) (x1 x5 5 5)})")
        val answer = fStp(stp)
        assert(answer == NIL)

        val minimalInconsistentStpFinder = new MinimalInconsistentStpFinder
        val minIncStp = minimalInconsistentStpFinder(stp)
        assert(minIncStp.asCol.size == 3)
    }

    test("Minimal Inconsistent Allen Constraints Set") {
        val c = new Container()
        val parser = new Parser(c)

        val acs = parser.str("{(before A B (5 10)) (before B C (5 10)) (after  A C (5 10)) (after  A D (5 10))} (before C E (5 10))")
        val answer = fStp(fAllen2Stp(acs))
        assert(answer == NIL)

        val minimalInconsistentAllenFinder = new MinimalInconsistentAllenFinder
        val minIncAcs = minimalInconsistentAllenFinder(acs)
        assert(minIncAcs.asCol.size == 3)
    }
}

