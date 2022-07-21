package org.aiddl.common.scala.reasoning.resource

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation._
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.temporal.Timepoint

import org.aiddl.core.scala.representation.TermImplicits.term2Num

class FlexibilityLossFunction extends Function {
  def apply( peak: CollectionTerm, dom: CollectionTerm ): Term = {
    def Est = Timepoint.Est(dom)_
    def Lst = Timepoint.Lst(dom)_
    def Eet = Timepoint.Eet(dom)_
    def Let = Timepoint.Let(dom)_

    var added: Set[Term] = Set.empty
    val mcss: Iterable[(Term, Term)] = peak.flatMap( a => {
      added = added + a
      peak.collect({ case b if (!added.contains(b)) => {
        added = added + b
        (a, b)
      }})})

    var pcMin = Num(1.1)
    var pcList: List[Term] = Nil

    def pc( i: Term, j: Term ): Term = {
      val dMin = Est(j) - Let(i)
      val dMax = Lst(j) - Eet(i)
      if (dMin == dMax) NIL else {
        val pc: Num =
          if (dMin < Num(0.0) && dMax < Num(0.0)) Num(1.0)
          else if (dMin == InfNeg() && dMax == InfPos()) Num(0.5)
          else {
            (dMax.min(Num(0.0)) - dMin.min(Num(0.0))) / (dMax - dMin)
          }
        pcList = pc :: pcList
        pcMin = pc.min(pcMin)
        Tuple(pc, i, j)
      }
    }

    val values = mcss.flatMap( _ match { case (i, j) => {
      List(pc(i, j), pc(j, i)).filter( _ != NIL ) }})

    val k = pcList.foldLeft(Num(0.0))( (c, pc) => c + (Num(1.0) / (Num(1.0) + pc - pcMin) ))
    Tuple(
      if ( k != Num(0.0) ) Num(1.0) / k.asNum else Num(1.0),
      ListTerm(values.toSeq)
    )
  }

  def apply( acs: Term ): Term = this(acs(0).asCol, acs(1).asCol)
}
