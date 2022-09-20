package org.aiddl.common.scala.planning.state_variable.heuristic

import scala.collection.mutable.HashMap
import scala.collection.mutable.HashSet
import scala.annotation.tailrec

import org.aiddl.core.scala.function.InterfaceImplementation
import org.aiddl.core.scala.function.Initializable

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.function.Configurable
import org.aiddl.core.scala.function.Verbose

import org.aiddl.core.scala.container.Container

import org.aiddl.core.scala.representation._

import org.aiddl.core.scala.representation.TermImplicits._
import org.aiddl.core.scala.representation.BoolImplicits._
import org.aiddl.common.scala.planning.PlanningTerm
import org.aiddl.common.scala.planning.state_variable.ReachableOperatorEnumerator

import org.aiddl.core.scala.representation.TermImplicits._
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2SetTerm

class SumCostHeuristic extends Function with InterfaceImplementation with Initializable {
    val interfaceUri = Sym("org.aiddl.common.planning.state-variable.heuristic");

    var actions = SetTerm()
    var goal = SetTerm() 

    def init( args: Term ) = {
        val operators = args(PlanningTerm.Operators)
        val state = args(PlanningTerm.InitialState)
        this.goal = args(PlanningTerm.Goal)
        this.actions = operators
    }

    def apply( s: Term ): Term = {
        val delta_0 = new HashMap[Term, Num]
        val us = new HashSet[SetTerm] 
        us.add(s)
        s.asCol.foreach( p => delta_0.put(p, Num(0)) )
        compute(s, goal, delta_0, us)
    }

    def apply( os: SetTerm, s: SetTerm, g: SetTerm ): Num = {
        val delta_0 = new HashMap[Term, Num]
        val us = new HashSet[SetTerm] 
        us.add(s.asSet)
        s.asCol.foreach( p => delta_0.put(p, Num(0)) )
        this.actions = os
        compute(s, g, delta_0, us)
    }

    @tailrec
    private def compute( s: SetTerm, g: SetTerm, delta_0: HashMap[Term, Num], us: HashSet[SetTerm] ): Num = {
        var change = false
        var foundGoal = false
        val newU = new HashSet[SetTerm]
        val remU = new HashSet[SetTerm]

        actions.find( a => {
            val pre = a(PlanningTerm.Preconditions).asCol
            val eff = a(PlanningTerm.Effects).asCol
            var change_op = false

            us.find( u => {
                if ( u.containsAll(pre) && !u.containsAll(eff) ) {
                    eff.foreach( e => {
                        val sum = pre.foldLeft(Num(1))( (c, p) => c + delta_0(p) )
                        val cur = delta_0.getOrElse(e, InfPos())
                        if (cur > sum) {
                            delta_0.put(e, sum)
                            change = true
                            change_op = true
                        }
                    })
                    val u_eff = u.addAll(eff)
                    foundGoal = u_eff.containsAll(g)
                    if ( !foundGoal ) {
                        remU.add(u)
                        newU.add(u_eff)
                    }
                    foundGoal || change_op
                } else {
                    false
                }
            })
            foundGoal
        })
        if ( !(change && !foundGoal) ) {
            goal.foldLeft(Num(0))( (c, g) => delta_0.get(g) match { 
                case Some(v) => c + v 
                case None => InfPos() })
        } else {
            us ++= newU
            us --= remU
            compute(s, g, delta_0, us)
        }
    }  
}