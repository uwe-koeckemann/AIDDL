package org.aiddl.common.scala.planning.state_variable

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.function.Configurable
import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.search.{GraphSearch, TermGraphSearch}
import org.aiddl.common.scala.planning.state_variable.heuristic.{CausalGraphHeuristic, FastForwardHeuristic, SumCostHeuristic}
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.BoolImplicits.bool2Boolean

class ForwardSearchPlanIterator extends TermGraphSearch {
    val loggerName = "ForwardSearchPlanner"

    val f_h = new CausalGraphHeuristic
    val f_goal = new GoalTest
    val f_exp = new Expansion

    var groundOperators: Term = _

    override def init( p: Term ) = {
        val ground = new ReachableOperatorEnumerator
        groundOperators = ground(p)

        val pGround = p.asList.put(KeyVal(Operators, groundOperators))

        f_exp.init(groundOperators)
        f_h.init(pGround)
        f_goal.init(p(Goal))
        super.init(ListTerm(p(InitialState)))
    }
     
    override def h( n: Term ): Num = f_h(n).asNum
    override def isGoal( n: Term ): Boolean = f_goal(n).asBool.v
    override def expand( n: Term ): Seq[(Term, Term)] = f_exp.expand(n)
}
