package org.aiddl.common.scala.reasoning.resource

import org.aiddl.common.scala.Common
import org.aiddl.core.scala.util.StopWatch
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.resource.ResourceTerm.*
import org.aiddl.common.scala.reasoning.temporal.{AllenInterval2Stp, StpSolver}
import org.aiddl.common.scala.search.GenericTreeSearch
import org.aiddl.core.scala.function.{Function, Initializable}

class EstaScheduler extends GenericTreeSearch[Term, Term] with Function with Initializable {
  val peakCollector = new PeakCollector
  val linearMcsSampler = new LinearMcsSampler
  val ordering = new FlexibilityOrdering
  val ac2stp = new AllenInterval2Stp
  val solveStp = new StpSolver

  var acs: CollectionTerm = _
  var capacities: CollectionTerm = _
  var usages: CollectionTerm = _
  var dom: Term = _

  override def init( rcpsp: Term ) = {
    this.acs = rcpsp(Constraints).asCol
    capacities = rcpsp(Capacity).asCol
    usages = rcpsp(Usage).asCol
    dom = solveStp(ac2stp(acs))
    if ( dom == NIL ) this.failed = true
    super.reset
  }

  override def isConsistent: Boolean = {
    val c = acs.addAll(SetTerm(this.choice.toSet))
    val stp = ac2stp(c)
    dom = solveStp(stp)
    dom != NIL
  }

  override def expand: Option[Seq[Term]] = {
    val peaks = peakCollector(capacities, usages, dom.asCol).asCol
    if ( peaks.isEmpty ) None else Some(ordering(peaks, dom.asCol).asList.list)
  }

  override val nil: Term =
    SetTerm.empty

  override def assembleSolution(choice: List[Term]): Option[Term] =
    Some(SetTerm(choice.toSet))

  override def apply(rcpsp: Term): Term =
    this.init(rcpsp)
    this.search.getOrElse(Common.NIL)
}
