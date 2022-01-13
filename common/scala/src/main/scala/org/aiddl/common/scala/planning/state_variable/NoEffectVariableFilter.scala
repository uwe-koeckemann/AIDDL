package org.aiddl.common.scala.planning.state_variable

import scala.collection.mutable.Set
import scala.collection.mutable.Map
import scala.collection.mutable.HashMap
import scala.collection.mutable.HashSet

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.function.Configurable

import org.aiddl.core.scala.container.Container

import org.aiddl.common.scala.planning.PlanningTerm

import org.aiddl.core.scala.representation._

import org.aiddl.core.scala.representation.TermImplicits._
import org.aiddl.core.scala.representation.BoolImplicits._
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2SetTerm


class NoEffectVariableFilter extends Function {
  def apply( x: Term ): Term = x match {
    case Tuple(s: CollectionTerm, g: CollectionTerm, as: CollectionTerm) => {
      val changingStateVariables = as.flatMap(a => a.getOrPanic(PlanningTerm.Effects).asCol).toSet

      val s_new = SetTerm(s.filter( sva => changingStateVariables.contains(sva.key) ).toSet)
      val g_new = SetTerm(g.filter( sva => changingStateVariables.contains(sva.key) || !s.contains(sva)  ).toSet)
      val as_new = SetTerm(as.map( a => a.asTup.put(KeyVal(PlanningTerm.Preconditions, SetTerm(
        a(PlanningTerm.Preconditions).asCol.filter( sva => changingStateVariables.contains(sva.key) ).toSet
      )) )).toSet)

      Tuple(s_new, g_new, as_new)
    }
    case _ => ???
  }
}