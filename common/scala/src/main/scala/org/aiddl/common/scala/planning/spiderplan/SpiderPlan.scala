package org.aiddl.common.scala.planning.spiderplan

import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.planning.spiderplan.SpiderPlan.{AddAll, Inconsistent}

object SpiderPlan {
  val Consistent = Sym("consistent")
  val Inconsistent = Sym("inconsistent")
  val Searching = Sym("searching")

  val AddAll = Sym("add-all")
  val PutAll = Sym("put-all")
  val Replace = Sym("replace")
  val Substitute = Sym("substitute")

  val Statement = Sym("statement")
  val Temporal = Sym("temporal")
  val PropagatedValue = Sym("propagated-value")
  val OpenGoal = Sym("goal")
  val Operator = Sym("operator")
  val ResourceReusableUsage = Sym("resource.reusable.usage")
  val ResourceReusableCapacity = Sym("resource.reusable.capacity")
}

trait SpiderPlan extends Function with Verbose {
  var generateAllSolutions = false

  val solvers: Vector[ResolverGenerator]

  def apply(args: Term): Term = {
    import SpiderPlan._
    var isConsistent = true

    var backtrackStack: List[ResolverIterator] = Nil
    var coreStack: List[CollectionTerm] = Nil
    //var resolverStack: List[Term] = Nil

    var currentCore = args.asCol

    var skipToNextResolver = false
    var done = false
    while { !done } do {
      var i = 0
      while { !done && i < solvers.size } do {
        if ( !skipToNextResolver ) {
          this.log(1, s"Running $i: ${solvers(i).getClass.getSimpleName}")
          val solver_i = this.solvers(i)
          val rs = solver_i(currentCore)
          coreStack = currentCore :: coreStack
          backtrackStack = rs :: backtrackStack
        } else {
          skipToNextResolver = false
        }

        this.log(1, "Choosing next resolver")
        var usedBacktracking = false
        var r: Term = NIL
        while { !backtrackStack.isEmpty && (r == NIL || r(0) == Inconsistent) } do {
          this.logInc(1, s"trying stack level ${backtrackStack.size}")
          r = backtrackStack.head.next
          this.logDec(1, s"Resolver: $r")
          if (r == NIL) {
            i = 0
            usedBacktracking = true
            //resolverStack = resolverStack.tail
            backtrackStack = backtrackStack.tail
            coreStack = coreStack.tail
          }
        }
        r match {
          case NIL => {
            isConsistent = false
            done = true
          }
          case Tuple(result, changes: CollectionTerm) => {
            currentCore = changes.foldLeft(currentCore: CollectionTerm)(
              (c, inst) => {
                inst match {
                  case Tuple(AddAll, key, cs: CollectionTerm) => c.put(KeyVal(key, c.getOrElse(key, SetTerm.empty).asCol.addAll(cs)))
                  case Tuple(PutAll, key, cs: CollectionTerm) => c.put(KeyVal(key, c.getOrElse(key, SetTerm.empty).asCol.putAll(cs)))
                  case Tuple(Replace, key, cs: CollectionTerm) => c.put(KeyVal(key, cs))
                  case Tuple(Substitute, s: CollectionTerm) => (c \ Substitution.from(s.asCol)).asCol
                  case _ => throw new IllegalArgumentException(s"Resolver instruction not supported: ${i}")
                }
              }
            )
            //resolverStack = r :: resolverStack
            if ( !usedBacktracking && result == Consistent ) {
              i += 1
            } else {
              i = 0
            }
          }
          case _ => throw new IllegalArgumentException(s"Unsupported resolver iterator result: ${r}")
        }
      }
      this.log(1, s"Done with i=${i} and done=$done")
      if ( !generateAllSolutions ) { done = true }
      else if ( !isConsistent ) { done = true }
      else { skipToNextResolver = true }
    }
    currentCore
  }
}
