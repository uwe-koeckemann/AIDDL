package org.aiddl.example.learning_agent_scala.function

import org.aiddl.common.scala.Common
import org.aiddl.common.scala.learning.LearningTerm
import org.aiddl.common.scala.planning.PlanningTerm
import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.{KeyVal, ListTerm, SetTerm, Sym, Term, Tuple, Var}

import scala.util.Random

class DataGoalGenerator extends Function with Verbose {
  val r = Random

  override def apply(x: Term): Term = {
    val locations = x(Sym("locations")).asCol
    val configs = x(Sym("configs")).asCol
    val data = x(LearningTerm.Data).asCol

    var goalCandidates: Vector[Term] = Vector.empty

    for ( loc <- locations ) {
      for ( cfg <- configs ) {
        val matchTerm = Tuple(loc, cfg, Var())
        if (!data.containsUnifiable(matchTerm)) {
          goalCandidates = goalCandidates.appended( KeyVal(Sym("collected"), Tuple(Sym("data"), loc, cfg)) )
        }
      }
    }
    val goalSelected = goalCandidates(r.nextInt(goalCandidates.size))

    SetTerm(goalSelected)
  }

}
