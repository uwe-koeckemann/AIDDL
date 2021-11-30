import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation.Var
import org.aiddl.core.scala.representation.Num
import org.aiddl.core.scala.parser.Parser
import org.aiddl.common.scala.math.linear_algebra.Matrix
import org.aiddl.common.scala.planning.state_variable.ReachableOperatorEnumerator
import org.aiddl.common.scala.planning.state_variable.ProblemCompiler
import org.aiddl.common.scala.planning.PlanningTerm._
import org.aiddl.common.scala.planning.state_variable.heuristic.SumCostHeuristic
import org.aiddl.core.scala.representation.TermImplicits._
import org.aiddl.core.scala.representation.InfPos
import org.aiddl.common.scala.planning.state_variable.ForwardSearchPlanIterator

class PlannerSuite extends AnyFunSuite {

    val p01 = {
        val c = new Container()
        val m = Parser.parseInto("../test/planning/state-variable/elevator/problem-01.aiddl", c)
        c.resolve(c.getEntry(m, Sym("problem")).get.v)
    }

    val p02 = {
        val c = new Container()
        val m = Parser.parseInto("../test/planning/state-variable/elevator/problem-02.aiddl", c)
        c.resolve(c.getEntry(m, Sym("problem")).get.v)
    }

    val p03 = {
        val c = new Container()
        val m = Parser.parseInto("../test/planning/state-variable/elevator/problem-03.aiddl", c)
        c.resolve(c.getEntry(m, Sym("problem")).get.v)
    }

    val p04 = {
        val c = new Container()
        val m = Parser.parseInto("../test/planning/state-variable/elevator/problem-03.aiddl", c)
        c.resolve(c.getEntry(m, Sym("problem")).get.v)
    }

    val forwardPlanner = new ForwardSearchPlanIterator

    test("Sum Cost heuristic value test 01") {
        //forwardPlanner.setVerbose("Plan", 2)
        forwardPlanner.init(p01)
        val plan = forwardPlanner.search
    }
}