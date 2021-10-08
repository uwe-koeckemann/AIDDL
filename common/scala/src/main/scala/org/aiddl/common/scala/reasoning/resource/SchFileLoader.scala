package org.aiddl.common.scala.reasoning.resource

import scala.io.Source
import scala.collection.mutable.HashMap

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation._
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.temporal.Timepoint
import org.aiddl.common.scala.reasoning.temporal.UnaryConstraint.Duration
import org.aiddl.common.scala.reasoning.temporal.IntervalDistanceConstraint.StSt
import org.aiddl.common.scala.reasoning.resource.ResourceTerm._

import org.aiddl.core.scala.representation.TermImplicits._

class SchFileLoader extends Function {

  def apply( fName: String ) = {
    var constraints: Set[Term] = Set.empty
    var capacities: Set[Term] = Set.empty
    var usages: Set[Term] = Set.empty
    var usageMap: HashMap[Term, Set[Term]] = new HashMap
    var i = 1
    var numActivities = 0
    var numResources = 0

    for (line <- Source.fromFile(fName).getLines) {
      val elems = line.replace("\t", " ").trim().split(" ")
      if ( i == 1 ) {
        numActivities = elems(0).toInt
        numResources = elems(1).toInt
      } else if ( i <= numActivities + 3 ) {
        val from = Sym(s"a${elems(0)}")
        val count = elems(2).toInt
        for ( j <- 3 until (count+3) ) {
          val to = Sym(s"a${elems(j)}")
          val delta = Num( elems(j+count).substring(1).replace("]", "").toInt )
          constraints += Tuple(StSt, from, to, Tuple(delta, InfPos()))
        }
      } else if ( i > numActivities + 3 && i <= 2*numActivities + 3) {
        val duration = Num(elems(2).toInt)
        constraints += Tuple(Duration, Sym(s"a${elems(0)}"), Tuple(duration, duration))
        for ( j <- 1 to numResources ) {
          val usage = elems(2+j).toInt
          val resource = Sym(s"r$j")
          val activity = Sym(s"a${elems(0)}")
          if ( usage > 0) {
            val pre = usageMap.getOrElseUpdate(resource, Set.empty)
            usageMap.put(resource, pre + KeyVal(activity, usage))
          }
        }
      } else {
        for ( j <- 1 to numResources ) {
          val cap = Num(elems(j-1).toInt)
          val resource = Sym(s"r$j")
          capacities += KeyVal(resource, SetTerm(KeyVal(Sym("min"), 0), KeyVal(Sym("max"), cap)))
          usages += KeyVal(resource, SetTerm(usageMap(resource)))
        }
      }
    }
    SetTerm(
      KeyVal(Capacity, SetTerm(capacities)),
      KeyVal(Usage, SetTerm(usages)),
      KeyVal(Constraints, SetTerm(constraints))
    )
  }

  def apply( args: Term ): Term = this(args)
}
