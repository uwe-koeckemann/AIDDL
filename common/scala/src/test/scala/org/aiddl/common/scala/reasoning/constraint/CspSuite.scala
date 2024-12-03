package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.graph.Graph2Dot
import org.aiddl.common.scala.math.graph.GraphType.Directed
import org.aiddl.common.scala.planning.PlanningTerm.{Constraints, Domains}
import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm.Variables
import org.aiddl.common.scala.reasoning.constraint.CspSolver
import org.aiddl.common.scala.reasoning.constraint.domain.{NQueensGenerator, Sudoku}
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.logger.Logger
import org.scalatest.funsuite.AnyFunSuite

import java.util.logging.Level

class CspSuite extends AnyFunSuite {
  val pMapColoring = {
    val c = new Container()
    val parser = new Parser(c)
    Function.loadDefaultFunctions(c)
    val m = parser.parseFile("aiddl-test/reasoning/constraint/map-coloring.aiddl")
    assert(c.typeCheckModule(m))
    c.eval(c.resolve(c.getEntry(m, Sym("test-1")).get.value))
  }

  val pQueens3 = {
    val c = new Container()
    val parser = new Parser(c)
    Function.loadDefaultFunctions(c)
    val m = parser.parseFile("aiddl-test/reasoning/constraint/3-queens.aiddl")
    assert(c.typeCheckModule(m))
    c.eval(c.resolve(c.getEntry(m, Sym("csp")).get.value))
  }

  val pQueens4 = {
    val c = new Container()
    val parser = new Parser(c)
    Function.loadDefaultFunctions(c)
    val m = parser.parseFile("aiddl-test/reasoning/constraint/4-queens.aiddl")
    assert(c.typeCheckModule(m))
    c.eval(c.resolve(c.getEntry(m, Sym("csp")).get.value))
  }

  val cspSolver = new CspSolver

  test("CSP solver on map coloing problem") {
    val a = cspSolver(pMapColoring)
    assert( NIL != a )
  }

  test("CSP solver on 3 queens problem") {
    val cspSolver = new CspSolver
    val a = cspSolver(pQueens3)
    assert( a == NIL )
  }

  test("CSP solver on 4 queens problem") {
    val a = cspSolver(pQueens4)
    assert( NIL != a )
  }

  test("CSP solver on n queens generated problem") {
    val cspSolver = new CspSolver
    val generate = new NQueensGenerator
    val csp = generate(Integer(10))
    val a = cspSolver(csp)
    assert( NIL != a )
  }

  test("CSP with two variables and no constraints enumerates all solutions") {
    val x = Var("x")
    val y = Var("y")
    val a = Sym("a")
    val b = Sym("b")
    val csp = new ConstraintSatisfactionProblem(
      List(x, y),
      Map(x -> List(a, b), y -> List(a, b)),
      Set.empty
    )
    val solver = new CspSolver
    solver.init(csp)
    val solutions = solver.iterator.toList
    assert(solutions.length == 4)
  }

  test("Testing Node Consistency") {
    val container = new Container
    val parser = new Parser(container)
    val cspTerm = ListTerm(
      KeyVal(Variables, SetTerm(Var("x"), Var("y"), Var("z"))),
      KeyVal(Domains,
        SetTerm(
          parser.str("?x:{1 2 3 4 5}"),
          parser.str("?y:{1 2 3 4 5}"),
          parser.str("?z:{1 2 3 4 5}")
        )),
      KeyVal(Constraints, SetTerm(
        Tuple(Tuple(Var("x")), FunRef(Sym("c1"), x => Bool(x(0).asNum < Num(3)))),
        Tuple(Tuple(Var("y")), FunRef(Sym("c2"), x => Bool(x(0).asNum == Num(3)))),
        Tuple(Tuple(Var("z")), FunRef(Sym("c3"), x => Bool(x(0).asNum != Num(3)))),
      ))
    )
    val csp = ConstraintSatisfactionProblem.fromTerm(cspTerm)
    val propagated = NodeConsistency(csp)

    assert(propagated.domains(Var("x")).length == 2)
    assert(propagated.domains(Var("y")).length == 1)
    assert(propagated.domains(Var("z")).length == 4)
  }

  test("Extract table from CSP constraints") {
    val generator = new NQueensGenerator

    val problem = generator(Num(5))
    val csp = ConstraintSatisfactionProblem.fromTerm(problem)

    val converter = new ConvertConstraintsToTables
    val tabulatedCsp = converter(csp)

    val solver1 = new CspSolver
    solver1.init(problem)

    val solver2 = new CspSolver
    solver2.init(problem)

    val result1 = solver1.search
    assert(result1.isDefined)

    val result2 = solver2.search
    assert(result2.isDefined)
  }

  test("Extract table from Sudoku constraints") {
    val known = ListTerm(
      Sudoku.assignment(1, 1, 5),
      Sudoku.assignment(1, 2, 3),
      Sudoku.assignment(1, 5, 7),

      Sudoku.assignment(2, 1, 6),
      Sudoku.assignment(2, 4, 1),
      Sudoku.assignment(2, 5, 9),
      Sudoku.assignment(2, 6, 5),

      Sudoku.assignment(3, 2, 9),
      Sudoku.assignment(3, 3, 8),
      Sudoku.assignment(3, 8, 6),

      Sudoku.assignment(4, 1, 8),
      Sudoku.assignment(4, 5, 6),
      Sudoku.assignment(4, 9, 3),

      Sudoku.assignment(5, 1, 4),
      Sudoku.assignment(5, 4, 8),
      Sudoku.assignment(5, 6, 3),
      Sudoku.assignment(5, 9, 1),

      Sudoku.assignment(6, 1, 7),
      Sudoku.assignment(6, 5, 2),
      Sudoku.assignment(6, 9, 6),

      Sudoku.assignment(7, 2, 6),
      Sudoku.assignment(7, 7, 2),
      Sudoku.assignment(7, 8, 8),

      Sudoku.assignment(8, 4, 4),
      Sudoku.assignment(8, 5, 1),
      Sudoku.assignment(8, 6, 9),
      Sudoku.assignment(8, 9, 5),

      Sudoku.assignment(9, 5, 8),
      Sudoku.assignment(9, 8, 7),
      Sudoku.assignment(9, 9, 9),
    )
    val creator = new Sudoku(known)
    val tableConverter = new ConvertConstraintsToTables
    val binaryConverter = new ConvertTablesToBinary
    val ac3 = new ArcConsistency3

    val problem = creator(known)

    var csp = ConstraintSatisfactionProblem.fromTerm(problem)

    var solver1 = new CspSolver
    solver1.init(csp)

    val result1 = solver1.search
    assert(result1.isDefined)

    csp = tableConverter.apply(csp)

    csp = binaryConverter(csp)

    solver1 = new CspSolver {
      checkWithGroundArgsOnly = true
    }
    solver1.init(csp)
    val result2 = solver1.search
    assert(result2.isDefined)

    solver1 = new CspSolver {
      checkWithGroundArgsOnly = true
    }

    csp = ac3(csp)
    solver1.init(csp)

    val result3 = solver1.search
    assert(result3.isDefined)
  }
}