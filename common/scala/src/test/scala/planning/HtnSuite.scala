import org.aiddl.common.scala.Common
import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation.Var
import org.aiddl.core.scala.representation.Num
import org.aiddl.core.scala.parser.Parser
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.linear_algebra.Matrix
import org.aiddl.common.scala.planning.state_variable.ReachableOperatorEnumerator
import org.aiddl.common.scala.planning.state_variable.ProblemCompiler
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.state_variable.heuristic.{CausalGraphHeuristic, FastForwardHeuristic, SumCostHeuristic}
import org.aiddl.common.scala.planning.task_network.TotalOrderForwardDecomposition
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.InfPos

class HtnSuite extends AnyFunSuite {
  val p01 = {
    val c = new Container()
    val m = Parser.parseInto("../test/planning/task-network/dock-worker-robot/problem-01.aiddl", c)
    c.resolve(c.getEntry(m, Sym("problem")).get.v)
  }

  val p02 = {
    val c = new Container()
    val m = Parser.parseInto("../test/planning/task-network/dock-worker-robot/problem-02.aiddl", c)
    c.resolve(c.getEntry(m, Sym("problem")).get.v)
  }

  val p03 = {
    val c = new Container()
    val m = Parser.parseInto("../test/planning/task-network/dock-worker-robot/problem-03.aiddl", c)
    c.resolve(c.getEntry(m, Sym("problem")).get.v)
  }

  val toDecomp = new TotalOrderForwardDecomposition

  test("Total-order Decomposition - Problem 01") {
    toDecomp.init(p01)
    var s = toDecomp(p01)
    assert( s.length == 4 )
  }

  test("Total-order Decomposition - Problem 02") {
    toDecomp.init(p02)
    var s = toDecomp(p02)
    assert( s != NIL )
  }

  test("Total-order Decomposition - Problem 03") {
    toDecomp.init(p03)
    var s = toDecomp(p03)
    assert( s == NIL )
  }
}