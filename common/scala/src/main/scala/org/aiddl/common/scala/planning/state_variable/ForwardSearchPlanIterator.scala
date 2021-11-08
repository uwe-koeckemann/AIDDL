package org.aiddl.common.scala.planning.state_variable

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.function.Configurable
import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.search.GraphSearch
import org.aiddl.common.scala.planning.state_variable.heuristic.{CausalGraphHeuristic, FastForwardHeuristic, SumCostHeuristic}
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.BoolImplicits.bool2Boolean

class ForwardSearchPlanIterator extends GraphSearch {
    val loggerName = "ForwardSearchPlanner"

    val f_h = new CausalGraphHeuristic
    val f_goal = new GoalTest
    val f_exp = new Expansion

    override def init( p: Term ) = {
        val ground = new ReachableOperatorEnumerator
        val os = ground(p)
        f_exp.init(os)
        f_h.init(p)
        f_goal.init(p(Goal))
        super.init(ListTerm(p(InitialState)))
    }
     
    def h( n: Term ): Num = f_h(n)
    def isGoal( n: Term ): Boolean = f_goal(n).asBool.v
    def expand( n: Term ): Seq[Term] = f_exp(n).asCol.toSeq
}
