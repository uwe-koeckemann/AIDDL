package org.aiddl.common.scala.reasoning.temporal

import org.aiddl.common.scala.Common
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{SetTerm, Term, Tuple}

class MinimalInconsistentStpFinder extends Function {

  override def apply(stp: Term): Term = {
    var cs = stp(1).asSet.toSet

    val solver = new StpSolver

    var change = true
    while ( change ) {
      change = false

      val removable = cs.find( c =>
        val csWithout = cs - c
        val stpTest = Tuple(stp(0), SetTerm(csWithout))
        solver(stpTest) == Common.NIL
      )

      change = removable match {
        case Some(c) => {
          cs = cs - c
          true
        }
        case None => false
      }
    }
    SetTerm(cs)
  }

}
