/*
package org.aiddl.common.scala.planning.temporal

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.InterfaceImplementation
import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.tools.Logger
import org.aiddl.common.scala.planning.PlanningTerm.{Effects, Id, Name, Operators, Plan, Preconditions, State}
import org.aiddl.common.scala.reasoning.temporal.UnaryConstraint.Duration
import org.aiddl.common.scala.reasoning.temporal.AllenConstraint.{Equals, Overlaps, During}


import scala.collection.mutable
import org.aiddl.core.scala.representation.TermImplicits.term2KeyVal

import scala.concurrent.duration.Duration

class Plan2AllenIntervalConverter extends Function with InterfaceImplementation with Verbose {
  val interfaceUri = Sym("org.aiddl.common.planning.temporal.plan2allen-interval")

  def apply(x: Term) : Term = {
    val state = x(State).asSet
    val plan = x(Plan).asList
    val operators = x(Operators).asSet

    val defaultBound = Tuple(Num(1), InfPos())

    val lastChange = new mutable.HashMap[Term, Term]
    var nextFreeId = 0

    state.forall( sva => {
      val i = Tuple(Sym(s"I$nextFreeId"), sva.key)
      nextFreeId += 1
      lastChange.put(sva.key, i)
    })
    var currentAction = 0
    Logger.++
    val aics = plan.flatMap( action => {
      currentAction += 1
      Logger.--
      this.logInc(1, s"$currentAction: $action")
      val op = operators.set.collectFirst( o => {o(Name) unify action} match {
        case Some(s) => o\s
      })
      val a = op match {
        case None => throw new IllegalArgumentException(s"No operator found for action $action")
        case Some(o) => {
          var o = o.get(Id) match {
            case Some(id) => {
              op \ new Substitution(id, Num(currentAction))
            }
            case None => o
          }
          if ( id != None ) {}
        }
      }
      val iAct = Tuple(Sym(s"A$currentAction"), action(0))
      var durAics = a.get(Duration) match {
        case Some(dur) => Set(Tuple(Duration, iAct, dur))
        case None => Set()
      }
      val preAics = a.get(Preconditions).map(p => {
        val iPre = Tuple(Sym(s"P$nextFreeId"), p.key)
        val iLast = lastChange(p.key)
        Tuple(Equals, iLast, iPre)
      }).toSet
      a.get(Effects).foreach( e => {
        val iEff = Tuple(Sym(s"E$nextFreeId"), e.key)
        lastChange.put(e.key, iEff)
      })

    })
  }
}
*/
