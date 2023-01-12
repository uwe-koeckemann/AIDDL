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

import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_KeyVal

import scala.language.implicitConversions


class NoEffectVariableFilter extends Function {
  def apply( x: Term ): Term = {
    val s = x(PlanningTerm.InitialState).asCol
    val g = x(PlanningTerm.Goal).asCol
    val as = x(PlanningTerm.Operators).asCol

    val changingStateVariables = as.flatMap(a => a.getOrPanic(PlanningTerm.Effects).asCol).toSet

    val s_new = SetTerm(s.filter( sva => changingStateVariables.exists(eff => eff.asKvp.key == sva.asKvp.key)).toSet)
    val g_new = SetTerm(g.filter( sva => changingStateVariables.exists(eff => eff.asKvp.key == sva.asKvp.key) || !s.contains(sva)  ).toSet)
    val as_new = SetTerm(as.map( a => a.asTup.put(KeyVal(PlanningTerm.Preconditions, SetTerm(
      a(PlanningTerm.Preconditions).asCol.filter( sva => changingStateVariables.exists( eff => eff.asKvp.key == sva.key) ).toSet
    )) )).toSet)

    SetTerm(
      KeyVal(PlanningTerm.InitialState, s_new),
      KeyVal(PlanningTerm.Goal, g_new),
      KeyVal(PlanningTerm.Operators, as_new))
  }
}