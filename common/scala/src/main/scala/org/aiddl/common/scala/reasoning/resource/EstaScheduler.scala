package org.aiddl.common.scala.reasoning.resource

import org.aiddl.core.scala.tools.StopWatch
import org.aiddl.core.scala.representation._
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.resource.ResourceTerm._
import org.aiddl.common.scala.reasoning.temporal.{AllenInterval2Stp, StpSolver}
import org.aiddl.common.scala.search.TreeSearch

import org.aiddl.core.scala.representation.TermImplicits._
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2CollectionTerm

class EstaScheduler extends TreeSearch {
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
    this.acs = rcpsp(Constraints)
    capacities = rcpsp(Capacity)
    usages = rcpsp(Usage)
    dom = solveStp(ac2stp(acs))
    if ( dom == NIL ) this.failed = true
    super.init(rcpsp)
  }

  override def isConsistent: Boolean = {
    val c = acs.addAll(SetTerm(this.choice.toSet))
    val stp = ac2stp(c)
    StopWatch.start("STP")
    dom = solveStp(stp)
    StopWatch.stop("STP")
    dom != NIL
  }

  override def expand: Option[Seq[Term]] = {
    val peaks = peakCollector(capacities, usages, dom)
    if ( peaks.isEmpty ) None else Some(ordering(peaks, dom).asList.list)
  }
}
