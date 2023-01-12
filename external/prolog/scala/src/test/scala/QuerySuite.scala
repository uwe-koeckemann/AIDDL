import org.aiddl.common.scala.Common
import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.util.logger.Logger
import org.aiddl.external.scala.prolog.QuerySolver

class QuerySuite extends AnyFunSuite {
  val c = new Container()
  val parser = new Parser(c)
  val m01 = parser.parseFile("../test/location-example/test-01.aiddl")

  test("Find all adjacent locations") {
    val query = c.getProcessedValueOrPanic(m01, Sym("query-1"))
    val querySolver = new QuerySolver
    val r = querySolver(query)
    assert(r.length == 3)
  }

  test("Find all in and out pairs") {
    val query = c.getProcessedValueOrPanic(m01, Sym("query-2"))
    val querySolver = new QuerySolver
    val r = querySolver(query)
    assert(r.length == 4)
  }

  test("Query is satisfiable without substitution") {
    val query = c.getProcessedValueOrPanic(m01, Sym("query-3"))
    val querySolver = new QuerySolver
    val r = querySolver(query)
    assert(r.length == 0)
  }

  test("Query without variables is not satisfiable") {
    val query = c.getProcessedValueOrPanic(m01, Sym("query-4"))
    val querySolver = new QuerySolver
    val r = querySolver(query)
    assert(r == Common.NIL)
  }

  test("Query with variables is not satisfiable") {
    val query = c.getProcessedValueOrPanic(m01, Sym("query-5"))
    val querySolver = new QuerySolver
    val r = querySolver(query)
    assert(r == Common.NIL)
  }
}
