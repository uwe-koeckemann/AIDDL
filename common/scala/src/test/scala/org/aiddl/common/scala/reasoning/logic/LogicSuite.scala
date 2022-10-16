package org.aiddl.common.scala.reasoning.logic

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.logic.{Cnf2DimacsConverter, KnowledgeBase2CnfConverter}
import org.aiddl.common.scala.reasoning.logic.propositional.DpllSolver
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.UnitTestRunner
import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.representation.TermParseImplicits.string2term

class LogicSuite extends AnyFunSuite {
    val sat = new DpllSolver
    val kb2cnf = new KnowledgeBase2CnfConverter
    val cnf2Dimacs = new Cnf2DimacsConverter

    test("Logic type unit tests working") {
        assert(UnitTestRunner.testFiles(scala.List("../test/reasoning/logic/test-cases.aiddl")))
    }

    test("Sat solver satisfiable problem 01") {
        val p = string2term("[[3 1 2] [3] [-1]]")
        val a = sat(p)
        assert(a == ListTerm(Num(-2), Num(3), Num(-1)))
    }

    test("Sat solver satisfiable problem 02") {
        val p = string2term("[[3 1 2] [3] [-1] [-2 -3]]")
        val a = sat(p)
        assert(a.asSet == SetTerm(Num(-2), Num(3), Num(-1)))
    }

    test("Sat solver unsatisfiable problem") {
        val p = string2term("[[3 1 2] [3] [-1] [-2 -3] [1 2]]")
        val a = sat(p)
        assert(a == NIL)
    }

    test("Convert knowledge base to CNF") {
        val p = string2term("[a (=> a b) (or (and a c) (and a d)) (xor c d)]")
        val answer = string2term("[[a] [(not a) b] [a d] [c a] [c d] [c (not c)] [(not d) (not c)] [(not d) d]]")
        val cnf = kb2cnf(p)
        assert(cnf == answer)
    }

    test("Convert knowledge base to CNF then to Dimacs") {
        val p = string2term("{a (=> a b) (or (and a c) (and a d)) (xor c d)}")
        val answer = string2term("{{(not d) d} {a} {a d} {c a} {c d} {(not a) b} {c (not c)} {(not d) (not c)}}")
        val cnf = kb2cnf(p)
        val dimacs = cnf2Dimacs.encode(cnf)
        val cnfBack = cnf2Dimacs.decode(dimacs)
    }
}