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

import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_SetTerm

import scala.language.implicitConversions


class ProblemCompiler extends Function {
  var invMap: Term = _
  def apply( x: Term ): Term = {
    val s = x (PlanningTerm.InitialState).asCol
    val g = x (PlanningTerm.Goal).asCol
    val as = x (PlanningTerm.Operators).asCol

    val valueMap = new HashMap[Term, Term]
    val s0_new = doSet(s, valueMap)
    val g_new = doSet(g, valueMap)
    val as_new = SetTerm(as.map( a => {
      val name_new = valueMap.getOrElseUpdate(a(PlanningTerm.Name), Num(valueMap.size))
      val pre_new = doSet(a(PlanningTerm.Preconditions), valueMap)
      val eff_new = doSet(a(PlanningTerm.Effects), valueMap)
      Tuple(
        KeyVal(PlanningTerm.Name, name_new),
        KeyVal(PlanningTerm.Preconditions, pre_new),
        KeyVal(PlanningTerm.Effects, eff_new))
    }).toSet)

    invMap = SetTerm(valueMap.map((k, v) => KeyVal(v, k)).toSet)

    SetTerm(
      KeyVal(PlanningTerm.InitialState, s0_new),
      KeyVal(PlanningTerm.Goal, g_new),
      KeyVal(PlanningTerm.Operators, as_new))
  }

  def doSet( S: SetTerm, valueMap: Map[Term, Term]) = {
    SetTerm(S.map( e => e match {
      case KeyVal(k, v) => {
        val k_new = valueMap.getOrElseUpdate(k, Num(valueMap.size))
        val v_new = valueMap.getOrElseUpdate(v, Num(valueMap.size))
        KeyVal(k_new, v_new)
      }
      case _ => ???
    }).toSet)
  }
}