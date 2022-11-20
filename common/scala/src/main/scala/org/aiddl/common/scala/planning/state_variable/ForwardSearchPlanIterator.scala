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
import org.aiddl.core.scala.util.StopWatch

class ForwardSearchPlanIterator extends TermGraphSearch {
    val loggerName = "ForwardSearchPlanner"

    val f_h = new CausalGraphHeuristic
    val f_goal = new GoalTest
    val f_exp = new Expansion

    var groundOperators: Term = _
    var needGrounding: Boolean = true

    override def init( p: Term ) = {
        var problem = p
        if needGrounding then {
            val ground = new ReachableOperatorEnumerator
            groundOperators = ground(problem)
            problem = problem.asCol.put(KeyVal(Operators, groundOperators))
        } else {
            groundOperators = problem(Operators)
        }


        f_exp.init(groundOperators)
        f_h.init(problem)
        f_goal.init(p(Goal))
        super.init(ListTerm(p(InitialState)))
    }
     
    override def h( n: Term ): Num = f_h(n).asNum
    override def isGoal( n: Term ): Boolean = f_goal(n).asBool.v
    override def expand( n: Term ): Seq[(Term, Term)] = f_exp.expand(n)
}
