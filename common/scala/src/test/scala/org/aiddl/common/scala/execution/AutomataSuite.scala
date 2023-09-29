package org.aiddl.common.scala.execution

import org.aiddl.common.scala.Common
import org.aiddl.common.scala.execution.automata.DeterministicFiniteStateMachine
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.{Bool, ListTerm, Sym, Term, Tuple, Var}
import org.aiddl.core.scala.util.UnitTestRunner
import org.scalatest.funsuite.AnyFunSuite

class AutomataSuite extends AnyFunSuite {
  test("Automata type unit tests working") {
    assert(UnitTestRunner.testFiles(scala.List("aiddl-test/execution/automata/test-cases.aiddl")))
  }

  test("Loading and advancing a state machine") {
    val c = new Container()
    val parser = new Parser(c)
    val m = parser.parseFile("aiddl-test/execution/automata/dfa-01.aiddl")
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