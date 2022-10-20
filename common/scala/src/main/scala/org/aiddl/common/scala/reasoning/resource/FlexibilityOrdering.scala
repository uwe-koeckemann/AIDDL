package org.aiddl.common.scala.reasoning.resource

import org.aiddl.core.scala.function.{Function, InterfaceImplementation}
import org.aiddl.core.scala.representation._
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.temporal.Timepoint
import org.aiddl.common.scala.reasoning.temporal.AllenConstraint.Before

import Term.given_Conversion_Term_Num
import scala.language.implicitConversions

class FlexibilityOrdering extends Function with InterfaceImplementation {
  val interfaceUri = Sym("org.aiddl.common.reasoning.resource.variable-value-ordering")
  val flexLoss = new FlexibilityLossFunction

  def apply( peaks: CollectionTerm, dom: CollectionTerm ): Term = {
    val variable = peaks.map( peak => flexLoss(peak.asCol, dom) ).maxBy( kPc => kPc(0).asNum )
    val values = variable(1).asList.sortWith( _(0) < _(0) )
    ListTerm(values.map( pair => Tuple(Before, pair(1), pair(2), Tuple(Num(0), InfPos())) ))
  }

  def apply( args: Term ): Term = this(args(0).asCol, args(1).asCol)
}