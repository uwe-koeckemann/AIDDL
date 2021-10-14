package org.aiddl.common.scala.planning.spiderplan

import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation._
import org.aiddl.common.scala.Common.NIL

object SpiderPlan {
  val Consistent = Sym("consistent")
  val Searching = Sym("searching")

  val Temporal = Sym("temporal")
  val IntervalDomains = Sym("interval-domains")
  val OpenGoals = Sym("open-goals")
  val ResourceUsage = Sym("resource-usage")
  val ResourceCapacities = Sym("resource-capacities")
}

trait SpiderPlan extends Function with Verbose {
  var generateAllSolutions = false

  val solverNames: Vector[Term]
  val solvers: Vector[ResolverGenerator]

  def apply(args: Term): Term = {
    var isConsistent = true

    var backtrackStack: List[Function] = Nil
    var coreStack: List[SetTerm] = Nil
    var resolverStack: List[SetTerm] = Nil

    var currentCore = args.asSet

    var skipToNextResolver = false
    var done = false
    while { !done } do {
      var i = 0
      while { i < solvers.size } do {
        if ( !skipToNextResolver ) {
          val solver_i = this.solvers(i)
          val rs = solver_i(currentCore)
          coreStack = currentCore :: coreStack
          backtrackStack = rs :: backtrackStack
        } else {
          skipToNextResolver = false
        }

        var usedBacktracking = false
        var r: Term = NIL
        while { !backtrackStack.isEmpty && (r == NIL || r(0) == Sym("inconsistent")) } do {
          r = backtrackStack.head(Sym("next"))
        }
      }
    }
    Sym("NIL")
  }
}
