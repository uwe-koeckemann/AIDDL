package org.aiddl.common.scala.planning.state_variable

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.state_variable.heuristic.SumCostHeuristic
import org.aiddl.common.scala.planning.state_variable.{ApplicableFunction, StateTransition}
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Configurable, Function, Initializable, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.ComboIterator

import scala.language.implicitConversions

class OperatorStateEnumerator extends Function {

  def apply( state: SetTerm, os: SetTerm ): SetTerm = {
    val app = os.flatMap( o => {
      val choices: List[List[Substitution]] = o(Preconditions).asCol.map( p => {
        state.map( s => (p.asKvp.key unify s.asKvp.key).flatMap(sub => sub + (p.asKvp.value unify s.asKvp.value)) )
          .collect( { case Some(sub) => sub } ).toList
      }).toList
      val cIter = new ComboIterator[Substitution](choices)
      cIter.map( combo => {
        val sub = combo.foldLeft(Some(new Substitution()): Option[Substitution])( (c, s) => c.flatMap( _ + s ) )
        sub match {
          case Some(s) => Some(o \ s)
          case None => None
        }
      }).collect( { case Some(a) => a } )
    }).toSet
    SetTerm(app)
  }

  def apply( args: Term ): Term = this(args(InitialState).asSet, args(Operators).asSet)
}
