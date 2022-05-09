package org.aiddl.common.scala.automata

import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation.Var
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.ListTerm
import org.aiddl.core.scala.representation.Bool
import org.aiddl.core.scala.representation.Term

import org.aiddl.core.scala.tools.UnitTestRunner

import org.aiddl.common.scala.automata.DeterministicFiniteStateMachine

class AutomataSuite extends AnyFunSuite {
  test("Automata type unit tests working") {
    assert(UnitTestRunner.testFiles(scala.List("../test/automata/test-cases.aiddl")))
  }

  test("Loading and advancing a state machine") {
    val c = new Container()
    val m = Parser.parseInto("../test/automata/dfa-01.aiddl", c)
    val p = c.getProcessedValueOrPanic(m, Sym("dfa"))

    assert(c.typeCheckModule(m))

    val f_DFS = new DeterministicFiniteStateMachine
    f_DFS.init(p)

    val s1 = f_DFS(Tuple(Sym("step"), Sym("a")))
    assert(s1 == Sym("s1"))

    val s2 = f_DFS(Tuple(Sym("multi-step"), ListTerm(Sym("a"), Sym("a"), Sym("a"))))
    assert(s2 == Sym("s1"))

    val s3 = f_DFS(Tuple(Sym("step"), Sym("b")))
    assert(s3 == Sym("s2"))
    assert(f_DFS(Sym("current-state")) == Sym("s2"))
    assert(f_DFS(Sym("is-final-state")) == Bool(true))
  }
}