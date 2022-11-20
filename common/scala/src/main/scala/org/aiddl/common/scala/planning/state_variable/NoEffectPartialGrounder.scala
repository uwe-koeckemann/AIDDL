package org.aiddl.common.scala.planning.state_variable

import org.aiddl.common.scala.planning.PlanningTerm
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Configurable, Function, Initializable}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_KeyVal
import org.aiddl.core.scala.util.ComboIterator

import scala.language.implicitConversions


class NoEffectPartialGrounder extends Function {
  def apply( x: Term ): Term = {
    val s = x(PlanningTerm.InitialState).asCol
    val g = x(PlanningTerm.Goal).asCol
    val as = x(PlanningTerm.Operators).asCol

    val changingStateVariables = SetTerm(as.flatMap(a => a.getOrPanic(PlanningTerm.Effects).asCol.map(_.asKvp.key(0))).toSet)

    val s_new = SetTerm(s.filter( sva => changingStateVariables.contains(sva.asKvp.key(0)) ).toSet)
    val g_new = SetTerm(g.filter( sva => changingStateVariables.contains(sva.asKvp.key(0)) || !s.contains(sva)  ).toSet)
    val as_new = SetTerm(as.flatMap( a => partiallyGround(a, s, changingStateVariables)).toSet)

    SetTerm(
      KeyVal(PlanningTerm.InitialState, s_new),
      KeyVal(PlanningTerm.Goal, g_new),
      KeyVal(PlanningTerm.Operators, as_new))
  }

  private def partiallyGround(o: Term, s: CollectionTerm, changing: CollectionTerm): Set[Term] = {
    val choices = o(PlanningTerm.Preconditions).asCol.filter( pre => !changing.contains(pre.asKvp.key(0)) ).map( pre => {
      s.flatMap( sva => {
        pre unify sva
      }).toSeq
    }).toSeq

    val combo = new ComboIterator(choices)

    val subs = combo.flatMap( combo =>
      combo.foldLeft( Some(new Substitution()): Option[Substitution] )((c, a) => c.flatMap(c => c + a))
    )

    subs.map( o \ _ ).map( a => {
      a.asTup.put(KeyVal(PlanningTerm.Preconditions, SetTerm(
        a(PlanningTerm.Preconditions).asCol.filter( sva => changing.contains(sva.asKvp.key(0)) ).toSet
      )) )
    }).toSet
  }
}