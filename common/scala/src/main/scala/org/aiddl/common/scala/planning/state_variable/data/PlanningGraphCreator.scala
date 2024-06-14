package org.aiddl.common.scala.planning.state_variable.data

import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.state_variable.OperatorStateEnumerator
import org.aiddl.core.scala.function.{Function, InterfaceImplementation}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_KeyVal

import scala.language.implicitConversions


class PlanningGraphCreator extends  Function {
  private val applicableActionGenerator = new OperatorStateEnumerator

  def apply( os: SetTerm, s: SetTerm, g: SetTerm ): Term = {
    var layers: List[Set[Term]] = List(s.asSet.set)
    var usedActions: Set[Term] = Set.empty
    while ({
      var al = if ( layers.tail == Nil ) Set.empty else layers.tail.head
      var pl = layers.head

      // get most recent mutex sets

      val applicable = applicableActionGenerator(SetTerm(pl), os) // avoid SetTerm wrapping here

      // filter out non-mutex compliant actions
      val (nps, nas) = applicable
        .filter(!usedActions.contains(_))
        .foldLeft((Set.empty: Set[Term], Set.empty: Set[Term]))( (c, a) => c match {
          case (ps, as) => { (ps ++ a(Effects).asSet, as + a) } } )

      // update mutex sets

      val noop = pl.map( p => Tuple(KeyVal(Name, Tuple(Sym("NOOP"), p.key, p.value)), KeyVal(Preconditions, SetTerm(p)), KeyVal(Effects, SetTerm(p)))).toSet

      // add new mutex sets to layers
      layers = (pl ++ nps) :: (al ++ nas ++ noop) :: layers
      usedActions = usedActions ++ nas
      (nps.nonEmpty || nas.nonEmpty)  && !g.asSet.set.subsetOf(layers.head)
    })()
    ListTerm(layers.map(SetTerm(_)))
  }
  def apply( p: Term ): Term =
    this(p(Operators).asSet, p(InitialState).asSet, p(Goal).asSet)
}