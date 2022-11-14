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

    override def init( p: Term ) = {
        val ground = new ReachableOperatorEnumerator
        StopWatch.start("Grounding")
        groundOperators = ground(p)
        StopWatch.stop("Grounding")

        println(s"C1 ${ground.c1}")
        println(s"C2 ${ground.c2}")
        println(s"C3 ${ground.c3}")
        println(s"C4 ${ground.c4}")

        val pGround = p.asList.put(KeyVal(Operators, groundOperators))

        f_exp.init(groundOperators)
        StopWatch.start("H Init")
        f_h.init(pGround)
        StopWatch.stop("H Init")
        f_goal.init(p(Goal))
        super.init(ListTerm(p(InitialState)))
    }
     
    override def h( n: Term ): Num = f_h(n).asNum
    override def isGoal( n: Term ): Boolean = f_goal(n).asBool.v
    override def expand( n: Term ): Seq[(Term, Term)] = f_exp.expand(n)
}
