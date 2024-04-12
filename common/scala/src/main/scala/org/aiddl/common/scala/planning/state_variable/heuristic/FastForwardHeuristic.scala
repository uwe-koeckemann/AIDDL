package org.aiddl.common.scala.planning.state_variable.heuristic

import scala.collection.{immutable, mutable}
import scala.annotation.tailrec
import org.aiddl.core.scala.function.{Function, InterfaceImplementation, Initializable}
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.logger.Logger
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.state_variable.ReachableOperatorEnumerator
import org.aiddl.common.scala.planning.state_variable.data.RelaxedPlanningGraphCreator

import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_KeyVal

import scala.language.implicitConversions


class FastForwardHeuristic extends Function with InterfaceImplementation with Initializable with Heuristic {
  val interfaceUri = Sym("org.aiddl.common.planning.state-variable.heuristic")

  val createRpg = new RelaxedPlanningGraphCreator
  var g: SetTerm = _
  var as: SetTerm = _
  val rpgCache = new mutable.HashMap[Term, Term]()

  val Noop = Sym("NOOP")

  def init( args: Term ) = {
    rpgCache.clear()
    g = args(Goal).asSet
    as = args(Operators).asSet
  }

  def apply( s: Term ): Num = this(s.asSet, g)

  def apply( s: SetTerm, g: SetTerm ): Num = {
    val rpg = rpgCache.getOrElseUpdate(s, createRpg(as, s, g)).asList

    if ( !g.set.subsetOf(rpg.head.asSet.set) ) InfPos()
    else {
      val earliestLayer: mutable.HashMap[Term, Int] = new mutable.HashMap
      rpg
        .reverse
        .zipWithIndex
        .foreach( (l, i) => if ( i % 2 == 0 ) l.asSet.foreach( p => earliestLayer.getOrElseUpdate(p, i)) )
      val actionDifficulty = rpg
        .tail
        .head
        .asSet
        .map( a =>
          if (a(Name).isInstanceOf[Tuple] && a(Name)(0) == Noop) a -> -1
          else a -> a(Preconditions).asSet.foldLeft(0)( _ + earliestLayer(_) ) )
        .toMap
      var unsatGoals = g.asSet.filter( g => !s.containsKey(g.key) || s(g.key) != g.value )
      Num(backward(unsatGoals, rpg, actionDifficulty).size)
    }
  }

  def backward( g: Iterable[Term], rpg: ListTerm, diff: Map[Term, Int] ): Set[Term] = {
    rpg.list match {
      case _ :: Nil => Set.empty[Term]
      case _ :: as :: tail: List[Term] => {
        val (ngs, sas) = g.foldLeft((Set.empty[Term], Set.empty[Term]))((c, p) => {
          val selected: Term = as.asSet.set.filter(a => a(Effects).asCol.contains(p)).minBy(diff(_))
          (c(0) ++ selected(Preconditions).asSet, c(1) + selected)
        })
        sas ++ backward(ngs, ListTerm(tail), diff)
      }
    }
  }
}
