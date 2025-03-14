package org.aiddl.common.scala.reasoning.logic

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.logic.sat.{Cnf2DimacsConverter, DpllSolver, KnowledgeBase2CnfConverter}
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.UnitTestRunner
import org.scalatest.funsuite.AnyFunSuite

import java.util.logging.Level

class SatisfiabilitySuite extends AnyFunSuite {
    val parser = new Parser(new Container)
    val sat = new DpllSolver //{ traceFlag = true }
    val kb2cnf = new KnowledgeBase2CnfConverter
    val cnf2Dimacs = new Cnf2DimacsConverter

    test("Logic type unit tests working") {
        assert(UnitTestRunner.testFiles(scala.List("aiddl-test/reasoning/logic/test-cases.aiddl")))
    }

    test("Sat solver satisfiable problem 01") {
        val p = parser.str("[[3 1 2] [3] [-1]]")
        val a = sat(p)
        assert(a.asSet == SetTerm(Num(-2), Num(3), Num(-1)))
    }

    test("Sat solver satisfiable problem 02") {
        val p = parser.str("[[3 1 2] [3] [-1] [-2 -3]]")
        val a = sat(p)
        assert(a.asSet == SetTerm(Num(-2), Num(3), Num(-1)))
    }

    test("Sat solver unsatisfiable problem") {
        val sat = new DpllSolver {
            traceFlag = true
        }
        val p = parser.str("[[3 1 2] [3] [-1] [-2 -3] [1 2]]")
        val a = sat(p)
        assert(a == NIL)
    }

    test("Sat solver in a slightly larger problem") {
        val p = parser.str("  [[-7 3 -6] [-8 5 3 ] [-7 -1 -8] [-7 5 8 ] [-1 -5 7 ] [4 7 5 ] [-3 5 2 ] [5 7 3 ] [5 3 -6] [6 -2 -8] [-4 5 3 ] [-8 -7 1 ] [-9 -6 8 ] [-4 5 9 ] [9 -2 3 ] [-6 2 -8] [2 -3 8 ] [-5 -7 -1] [-2 4 -1] [-7 6 -5] [1 5 -3] [1 -4 -9] [-8 -4 -7] [-7 6 1 ] [-3 2 4 ] [-4 5 -2] [-9 3 2 ] [-6 5 -2] [4 1 -7] [-4 -5 -1] [5 9 1 ] [-3 -7 5 ] [1 -5 8 ] [-3 -2 -9] [-6 5 8 ] [6 7 -1] [-7 4 -5] [9 -7 6 ] [7 2 -1] [-7 6 1 ] [8 9 3 ] [9 -3 2 ] [6 2 -1] [3 8 6 ] [1 4 -3] [-1 -6 4 ] [-2 -7 -5] [-3 5 2 ] [-3 1 6 ] [-9 -4 -8] [-9 -5 2 ] [-6 7 9 ] [-2 7 3 ] [-5 8 4 ] [2 -4 -5] [-6 2 1 ] [9 6 -2] [-2 9 -3] [5 -2 -7] [-3 2 -7]]")

        val sat = new DpllSolver
        sat.init(p)
        val a = sat.search
        assert(a.isDefined)
    }

    test("Convert knowledge base to CNF - 01") {
        val p = parser.str("[a (=> a b) (or (and a c) (and a d)) (xor c d)]")
        val answer = parser.str("[[a] [(not a) b] [a d] [c a] [c d] [c (not c)] [(not d) (not c)] [(not d) d]]")
        val cnf = kb2cnf(p)
        assert(cnf == answer)
    }

    test("Convert knowledge base to CNF then to Dimacs") {
        val p = parser.str("{a (=> a b) (or (and a c) (and a d)) (xor c d)}")
        val answer = parser.str("{{(not d) d} {a} {a d} {c a} {c d} {(not a) b} {c (not c)} {(not d) (not c)}}")
        val cnf = kb2cnf(p)
        val dimacs = cnf2Dimacs.encode(cnf)
        val cnfBack = cnf2Dimacs.decode(dimacs)
    }

    test("Sat solver satisfiable problem with pure literal") {
        val p = parser.str("[[3 1 2] [3 -2] [3 -1]]")
        val a = sat(p)
        assert(a.asSet == SetTerm(Num(-2), Num(3), Num(-1)))
    }

    test("Convert knowledge base to CNF - 02") {
        val p = parser.str("[(<= a b) (not (and a b)) (not (or (not a) (not b)))]")
        val answer = parser.str("[[a (not b)] [(not a) (not b)] [a] [b]]")
        val cnf = kb2cnf(p)
        assert(cnf == answer)
    }

    test("Convert knowledge base to CNF - 03") {
        val p = parser.str("[(not (<= a b))]")
        val answer = parser.str("[[(not a)] [b]]")
        val cnf = kb2cnf(p)
        assert(cnf == answer)
    }
}