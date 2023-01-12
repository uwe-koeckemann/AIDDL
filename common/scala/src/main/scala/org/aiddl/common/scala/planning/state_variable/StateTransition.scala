package org.aiddl.common.scala.planning.state_variable

import scala.collection.mutable.Set
import scala.collection.mutable.Map
import scala.collection.mutable.HashMap
import scala.collection.mutable.HashSet

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.function.Configurable

import org.aiddl.core.scala.container.Container

import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.planning.PlanningTerm._

class StateTransition extends Function {
  def apply( x: Term ): Term = x match {
    case Tuple( a: Tuple, s: SetTerm ) => this(a, s)
    case _ => ???
  }

  def apply( a: Tuple, s: SetTerm ): Term = s.putAll(a(Effects).asCol)
}