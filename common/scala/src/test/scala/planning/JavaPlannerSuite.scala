import org.scalatest.funsuite.AnyFunSuite

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation.Var
import org.aiddl.core.scala.representation.Num
import org.aiddl.core.scala.parser.Parser

import org.aiddl.common.scala.math.linear_algebra.Matrix
import org.aiddl.common.planning.state_variable.ForwardSearchPlanner
import org.aiddl.core.scala.representation.TermImplicits._
import org.aiddl.common.scala.planning.state_variable.ReachableOperatorEnumerator
import org.aiddl.common.scala.planning.state_variable.ProblemCompiler
import org.aiddl.common.scala.planning.PlanningTerm
import org.aiddl.common.scala.tools.java.JavaFunctionWrapper

class JavaPlannerSuite extends AnyFunSuite {
    test("Enumerate reachable actions") {
        val c = new Container()
        val m = Parser.parseInto("../../../pub/common/test/planning/state-variable/elevator/problem-01.aiddl", c)
        val p = c.getEntry(m, Sym("problem")).get.v
        val p_res = c.resolve(p)

        val f_rae = new ReachableOperatorEnumerator()

        val as = f_rae(p_res)

        val f_pc = ProblemCompiler()

        val r = f_pc(Tuple(p_res(PlanningTerm.InitialState), p_res(PlanningTerm.Goal), as))
    }
}