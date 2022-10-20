package org.aiddl.common.scala.reasoning.resource

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation._
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.temporal.Timepoint

import Term.given_Conversion_Term_KeyVal
import Term.given_Conversion_Term_Num

import scala.language.implicitConversions

class PeakCollector extends Function {

  val linearMcsSampler = new LinearMcsSampler

  def apply( cap: CollectionTerm, usages: CollectionTerm, dom: CollectionTerm ): Term = {
    def Est = Timepoint.Est(dom)_
    def Lst = Timepoint.Lst(dom)_
    def Eet = Timepoint.Eet(dom)_
    def Let = Timepoint.Let(dom)_

    var peaks: List[Term] = Nil

    cap.foreach( c => {
      val resource = c.key
      val cap: Num = c.value(Sym("max"))
      val sortedUsages: Seq[Term] = usages.getOrElse(resource, ListTerm.empty).asList.sortWith( (a, b) => Est(a.key) < Est(b.key) ).toList

      var i, j = 0
      var current, st, et = Num(0)
      var peak: List[Term] = Nil

      while ( i < sortedUsages.length ) {
        if ( peak.isEmpty ) {
          val a_i = sortedUsages(i).key
          peak = a_i :: peak
          j = i + 1
          current = sortedUsages(i).value
          st = Est(a_i)
          et = Eet(a_i)
        } else {
          if ( j < sortedUsages.size ) {
            val a_j = sortedUsages(j).key
            val newSt = st.max(Est(a_j))
            val newEt = et.min(Eet(a_j))
            if ( newSt < newEt ) {
              peak = a_j :: peak
              st = newSt
              et = newEt
              current += sortedUsages(j).value

              if ( j == sortedUsages.size-1 && current > cap ) {
                peaks = linearMcsSampler(peak, usages(resource).asCol, cap) ++ peaks
                //peaks = SetTerm(peak.toSet) :: peaks
                i += 1
                peak = Nil
              }
            } else {
              if ( current > cap ) {
                peaks = linearMcsSampler(peak, usages(resource).asCol, cap) ++ peaks
                //peaks = SetTerm(peak.toSet) :: peaks
              }
              i += 1
              peak = Nil
            }
            j += 1
          } else {
            i += 1
            peak = Nil
          }
        }
      }
    })
    ListTerm(peaks.distinct)
  }

  def apply( args: Term ): Term = this(args(0).asCol, args(1).asCol, args(2).asCol)
}
