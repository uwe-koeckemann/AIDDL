package org.aiddl.common.scala.learning.supervised

import org.aiddl.core.scala.function.Function

import org.aiddl.core.scala.representation._
import org.aiddl.core.scala.container.Container

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.learning.Term._

import org.aiddl.core.scala.representation.TermCollectionImplicits.term2ListTerm

trait ScoreFunction extends Function {
  def score( y_p: ListTerm, y_t: ListTerm ): Num
  def apply( args: Term ): Term = score(args(0), args(1))
}