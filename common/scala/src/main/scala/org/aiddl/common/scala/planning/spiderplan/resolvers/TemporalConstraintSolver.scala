package org.aiddl.common.scala.planning.spiderplan.resolvers

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.planning.spiderplan.ResolverGenerator
import org.aiddl.common.scala.planning.spiderplan.ResolverIterator
import org.aiddl.common.scala.planning.spiderplan.ResolverSequenceIterator
import org.aiddl.common.scala.planning.spiderplan.SpiderPlan.*
import org.aiddl.common.scala.planning.spiderplan.resolvers.ReusableResourceScheduler
import org.aiddl.common.scala.reasoning.resource.{FlexibilityOrdering, LinearMcsSampler}
import org.aiddl.common.scala.reasoning.temporal.{AllenInterval2Stp, StpSolver}
import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.TermImplicits.term2KeyVal

import scala.collection.{immutable, mutable}

class TemporalConstraintSolver extends ResolverGenerator {
  override val targets: List[Sym] = List(Temporal)

  val ac2stp = new AllenInterval2Stp
  val stpSolver = new StpSolver

  def apply( cdb: Term ): ResolverIterator = {
    val acs = cdb.getOrElse(Temporal, SetTerm())
    val r = stpSolver(ac2stp(acs))
    println(s"Temporal result: $r")
    r match {
      case NIL => new ResolverSequenceIterator(false, List())
      case propVals => new ResolverSequenceIterator(true, List(ListTerm(List(Tuple(PutAll, PropagatedValue, propVals)))))
    }
  }
}