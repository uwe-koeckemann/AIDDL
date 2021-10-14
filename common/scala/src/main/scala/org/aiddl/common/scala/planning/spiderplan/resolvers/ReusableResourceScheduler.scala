package org.aiddl.common.scala.planning.spiderplan.resolvers

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.planning.spiderplan.ResolverGenerator
import org.aiddl.common.scala.planning.spiderplan.ResolverList
import org.aiddl.common.scala.planning.spiderplan.SpiderPlan.{IntervalDomains, ResourceCapacities, ResourceUsage}
import org.aiddl.common.scala.planning.spiderplan.resolvers.ReusableResourceScheduler.nextFreeId
import org.aiddl.common.scala.reasoning.resource.{FlexibilityOrdering, LinearMcsSampler}
import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.TermImplicits.term2KeyVal

import scala.collection.{immutable, mutable}

object ReusableResourceScheduler {
  var nextFreeId = 0
}

class ReusableResourceScheduler extends ResolverGenerator {
  override val targets: List[Sym] = List(Sym("resource-capacities"), Sym("resource-usages"))

  def init( args: Term ): Unit = {}

  def apply( cdb: Term ): Term = {
    val caps = cdb(ResourceCapacities).asSet
    val usagesEntries = cdb(ResourceUsage).asSet
    val intervalDomains = cdb(IntervalDomains)

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

    val sample = new LinearMcsSampler
    val peaks = sample(Tuple(groundCaps, usages, intervalDomains))

    if ( peaks.length == 0 ) {
      FunRef(Sym(s"#resource.resolver.$nextFreeId"), new ResolverList(true, List(SetTerm())))
    } else {
      val valueOrdering = new FlexibilityOrdering
      val resolvers = valueOrdering(Tuple(peaks, intervalDomains))
      nextFreeId += 1
      FunRef(Sym(s"#resource.resolver.$nextFreeId"), new ResolverList(false, resolvers.asList.toList))
    }
  }
}
