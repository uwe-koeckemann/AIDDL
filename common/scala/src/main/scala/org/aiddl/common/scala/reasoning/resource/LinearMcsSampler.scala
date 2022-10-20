package org.aiddl.common.scala.reasoning.resource

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation._
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.temporal.Timepoint

import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_Num

import scala.language.implicitConversions

class LinearMcsSampler  extends Function {

  def apply( peak: List[Term], usage: CollectionTerm, cap: Num ): List[Term] = {
    val sortedPeak = peak.sortWith( (a,b) => usage(a) < usage(b) )
    sample(peak, usage, cap)
  }

  def sample( peak: List[Term], usage: CollectionTerm, cap: Num ): List[Term] = {
    val next = takePeak(Nil, Num(0), peak, usage, cap)
    if ( next == Nil ) Nil
    else SetTerm(next.toSet) :: sample(peak.tail, usage, cap)
  }

  def takePeak( mcs: List[Term], s: Num, peak: List[Term], usage: CollectionTerm,  cap: Num ): List[Term] = {
    if ( s > cap ) mcs
    else if ( peak.isEmpty ) Nil
    else takePeak(peak.head :: mcs, s + usage(peak.head), peak.tail, usage, cap)
  }

  override def apply(x: Term): Term = ListTerm(this(x(0).asList.toList, x(1).asCol, x(2)))
}
