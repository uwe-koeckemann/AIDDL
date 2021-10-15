package planning

import org.aiddl.common.scala.Common
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.linear_algebra.Matrix
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.spiderplan.resolvers.{OpenGoalResolver, ReusableResourceScheduler, TemporalConstraintSolver}
import org.aiddl.common.scala.planning.state_variable.{ProblemCompiler, ReachableOperatorEnumerator}
import org.aiddl.common.scala.planning.state_variable.heuristic.{CausalGraphHeuristic, FastForwardHeuristic, SumCostHeuristic}
import org.aiddl.common.scala.planning.spiderplan.{ResolverGenerator, SpiderPlan}
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.tools.Logger
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.*
import org.scalatest.funsuite.AnyFunSuite

class SpiderplanSuite extends AnyFunSuite {
  val p01 = {
    val c = new Container()
    val m = Parser.parseInto("../test/planning/spiderplan/domain-01.aiddl", c)
    c.resolve(c.getEntry(m, Sym("problem")).get.v)
  }

  val spiderplan = new SpiderPlan {
    override val solvers: Vector[ResolverGenerator] = Vector(
      new TemporalConstraintSolver,
      new ReusableResourceScheduler,
      new OpenGoalResolver
    )
  }

  test("SpiderPlan - Problem 01") {
    spiderplan.setVerbose(2)
    var s = spiderplan(p01)
    println(Logger.prettyPrint(s, 1))
  }
}