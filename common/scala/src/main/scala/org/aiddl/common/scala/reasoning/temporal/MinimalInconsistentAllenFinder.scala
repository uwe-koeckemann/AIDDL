package org.aiddl.common.scala.reasoning.temporal

import org.aiddl.common.scala.Common
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{SetTerm, Term, Tuple}

class MinimalInconsistentAllenFinder extends Function {

  override def apply(acs: Term): Term = {
    var cs = acs.asSet.toSet

    val allenInterval2Stp = new AllenInterval2Stp
    val solver = new StpSolver

    var change = true
    while ( change ) {
      change = false

      val removable = cs.find( c =>
        val csWithout = cs - c
        val stpTest = allenInterval2Stp(SetTerm(csWithout))
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
