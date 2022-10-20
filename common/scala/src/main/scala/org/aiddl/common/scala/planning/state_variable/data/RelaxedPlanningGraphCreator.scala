package org.aiddl.common.scala.planning.state_variable.data

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.InterfaceImplementation
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.planning.PlanningTerm.{Name, Preconditions, Effects, InitialState, Goal, Operators}

import Term.given_Conversion_Term_KeyVal
import scala.language.implicitConversions

class RelaxedPlanningGraphCreator extends  Function with InterfaceImplementation  {
  val interfaceUri = Sym("org.aiddl.common.planning.state-variable.data.rpg-creator")

  def apply( os: SetTerm, s: SetTerm, g: SetTerm ): Term = {
    var layers: List[Set[Term]] = List(s.asSet.set)
    var unusedActions: Set[Term] = os.asSet.set
    while ({
      var al = if ( layers.tail == Nil ) Set.empty else layers.tail.head
      var pl = layers.head
      val (nps, nas) = unusedActions.filter( a => a(Preconditions).asSet.set.subsetOf(pl) )
        .foldLeft((Set.empty: Set[Term], Set.empty: Set[Term]))( (c, a) => c match {
          case (ps, as) => { (ps ++ a(Effects).asSet, as + a) } } )

      val noop = pl.map( p => Tuple(KeyVal(Name, Tuple(Sym("NOOP"), p.key, p.value)), KeyVal(Preconditions, SetTerm(p)), KeyVal(Effects, SetTerm(p)))).toSet
      layers = (pl ++ nps) :: (al ++ nas ++ noop) :: layers
      unusedActions = unusedActions -- nas
      (!nps.isEmpty || !nas.isEmpty)  && !g.asSet.set.subsetOf(layers.head)
    })()
    ListTerm(layers.map(SetTerm(_)))
  }
  def apply( p: Term ): Term = this(p(Operators).asSet, p(InitialState).asSet, p(Goal).asSet)
}