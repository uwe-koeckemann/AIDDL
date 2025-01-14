package org.aiddl.common.scala.planning.state_variable

import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.search.TermGraphSearch
import org.aiddl.common.scala.planning.state_variable.heuristic.{CausalGraphHeuristic, FastForwardHeuristic, Heuristic, SumCostHeuristic}
import org.aiddl.common.scala.planning.PlanningTerm.*

object ForwardSearchPlanIterator {
    def apply(): ForwardSearchPlanIterator ={
        new ForwardSearchPlanIterator(List((new CausalGraphHeuristic, Num(1))))
    }
}

class ForwardSearchPlanIterator(hs: Seq[(Heuristic, Num)]) extends TermGraphSearch {
    //val f_h = new CausalGraphHeuristic
    val f_goal = new GoalTest
    val f_exp = new Expansion

    hs.foreach((h, o) => this.addHeuristic(o, h))


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
        this.heuristics.foreach(h =>
            if  h.isInstanceOf[Initializable]
            then h.asInstanceOf[Initializable].init(problem))
        //f_h.init(problem)
        f_goal.init(p(Goal))
        super.init(ListTerm(p(InitialState)))
    }

    override def isGoal(node: Term ): Boolean = f_goal(node).asBool.v
    override def expand(node: Term ): Seq[(Term, Term)] = f_exp.expand(node)
}
