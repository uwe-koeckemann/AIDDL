package org.aiddl.common.scala.planning.spiderplan.resolvers

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.planning.spiderplan.{ResolverGenerator, ResolverIterator, ResolverSequenceIterator}
import org.aiddl.common.scala.planning.spiderplan.SpiderPlan.*
import org.aiddl.common.scala.planning.spiderplan.resolvers.ReusableResourceScheduler
import org.aiddl.common.scala.reasoning.resource.{FlexibilityOrdering, LinearMcsSampler, PeakCollector}
import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.TermImplicits.term2KeyVal

import scala.collection.{immutable, mutable}

class ReusableResourceScheduler extends ResolverGenerator {
  override val targets: List[Sym] = List(ResourceReusableUsage, ResourceReusableCapacity)

  def apply( cdb: Term ): ResolverIterator = {
    val caps = cdb(ResourceReusableCapacity).asSet
    val usagesEntries = cdb(ResourceReusableUsage).asSet
    val intervalDomains = cdb(PropagatedValue)

    val usageMap = new mutable.HashMap[Term, immutable.Set[Term]]()
    usagesEntries.foreach( u => {
      usageMap.put(u(1), usageMap.getOrElseUpdate(u(1), Set()) + KeyVal(u(0), u(2)))
    })
    var matchedCaps: Set[Term] = Set.empty
    val usages: SetTerm = SetTerm(usageMap.keys.map( r => {
      caps.collectFirst( c => {
        {r unify c.key} match {
          case Some(s) => c\s
        }
      } ) match {
        case Some(mc) => { matchedCaps = matchedCaps + mc }
        case None => throw new IllegalArgumentException(s"No capacity found for resource $r")
      }
      KeyVal(r, SetTerm(usageMap(r)))
    }).toSet)

    val groundCaps = SetTerm(matchedCaps)

    val sample = new PeakCollector
    val peaks = sample(Tuple(groundCaps, usages, intervalDomains))

    if ( peaks.length == 0 ) {
      new ResolverSequenceIterator(true, List(SetTerm()))
    } else {
      val valueOrdering = new FlexibilityOrdering
      val resolvers = valueOrdering(Tuple(peaks, intervalDomains)).asList.map( r => {
        Tuple(AddAll, Temporal, r)
      })

      new ResolverSequenceIterator(false, resolvers)
    }
  }
}
