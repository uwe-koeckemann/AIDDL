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

import org.aiddl.core.scala.representation.TermImplicits._
import org.aiddl.core.scala.representation.BoolImplicits._

import org.aiddl.core.scala.representation.TermCollectionImplicits.term2SetTerm
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2Tuple

class GoalTest extends Function with Initializable {
  var goal: SetTerm = SetTerm()

  def init( g: Term ): Unit = { this.goal = g }

  def apply( s: Term ): Term = Bool(goal.forall( g => s.get(g.key) match {
    case Some(v) => v == g.value
    case None => false } ))
}