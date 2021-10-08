package org.aiddl.common.scala.planning.state_variable

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.function.Configurable
import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.search.GraphSearch
import org.aiddl.common.scala.planning.state_variable.heuristic.SumCostHeuristic
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.state_variable.{ApplicableFunction, StateTransition}
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2SetTerm
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.BoolImplicits.bool2Boolean
import org.aiddl.core.scala.representation.BoolImplicits.term2Boolean
import org.aiddl.core.scala.tools.ComboIterator

class OperatorStateEnumerator extends Function {

  def apply( state: SetTerm, os: SetTerm ): SetTerm = {
    val app = os.flatMap( o => {
      val choices: List[List[Substitution]] = o(Preconditions).map( p => {
        state.map( s => (p.key unify s.key).flatMap(sub => sub + (p.value unify s.value)) )
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

  def apply( args: Term ): Term = this(args(InitialState), args(Operators))
}
