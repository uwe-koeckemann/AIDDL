package org.aiddl.common.scala.planning.state_variable

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.function.Configurable
import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.search.{GraphSearch, TermGraphSearch}
import org.aiddl.common.scala.planning.state_variable.heuristic.{CausalGraphHeuristic, FastForwardHeuristic, Heuristic, SumCostHeuristic}
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.core.scala.util.StopWatch

object ForwardSearchPlanIterator {
    def apply(): ForwardSearchPlanIterator ={
        new ForwardSearchPlanIterator(List((new CausalGraphHeuristic, Num(1.0))))
    }
}

class ForwardSearchPlanIterator(hs: Seq[(Heuristic, Num)]) extends TermGraphSearch {
    //val f_h = new CausalGraphHeuristic
    val f_goal = new GoalTest
    val f_exp = new Expansion


    var planningHeuristics: Vector[Heuristic] = Vector.empty // (f_h)//Vector.empty
    hs.foreach((h, o) => this.addHeuristic(h, o))


    var groundOperators: Term = _
    var needGrounding: Boolean = true

    def addHeuristic(h: Heuristic, omega: Num): Unit = {
        planningHeuristics = planningHeuristics.appended(h)
        super.addHeuristic(x => h(x), omega)
    }

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
        this.planningHeuristics.foreach(h =>
            if  h.isInstanceOf[Initializable]
            then h.asInstanceOf[Initializable].init(problem))
        //f_h.init(problem)
        f_goal.init(p(Goal))
        super.init(ListTerm(p(InitialState)))
    }
     
    //override def h( n: Term ): Num = f_h(n).asNum
    override def isGoal( n: Term ): Boolean = f_goal(n).asBool.v
    override def expand( n: Term ): Seq[(Term, Term)] = f_exp.expand(n)
}
