package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.container.Entry
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.constraint.CspSolver
import org.aiddl.common.scala.reasoning.constraint.domain.NQueensGenerator
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.scalatest.funsuite.AnyFunSuite

class CspSuite extends AnyFunSuite {
  val pMapColoring = {
    val c = new Container()
    val parser = new Parser(c)
    Function.loadDefaultFunctions(c)
    val m = parser.parseFile("../test/reasoning/constraint/map-coloring.aiddl")
    assert(c.typeCheckModule(m))
    c.eval(c.resolve(c.getEntry(m, Sym("test-1")).get.v))
  }

  val pQueens3 = {
    val c = new Container()
    val parser = new Parser(c)
    Function.loadDefaultFunctions(c)
    val m = parser.parseFile("../test/reasoning/constraint/3-queens.aiddl")
    assert(c.typeCheckModule(m))
    c.eval(c.resolve(c.getEntry(m, Sym("csp")).get.v))
  }

  val pQueens4 = {
    val c = new Container()
    val parser = new Parser(c)
    Function.loadDefaultFunctions(c)
    val m = parser.parseFile("../test/reasoning/constraint/4-queens.aiddl")
    assert(c.typeCheckModule(m))
    c.eval(c.resolve(c.getEntry(m, Sym("csp")).get.v))
  }

  val cspSolver = new CspSolver

  test("CSP solver on map coloing problem") {
    val a = cspSolver(pMapColoring)
    assert( NIL != a )
  }

  test("CSP solver on 3 queens problem") {
    val a = cspSolver(pQueens3)
    assert( NIL == a )
  }

  test("CSP solver on 4 queens problem") {
    val a = cspSolver(pQueens4)
    assert( NIL != a )
  }

  test("CSP solver on n queens generated problem") {
    val generate = new NQueensGenerator
    val csp = generate(Integer(10))
    val a = cspSolver(csp)
    assert( NIL != a )
  }
}