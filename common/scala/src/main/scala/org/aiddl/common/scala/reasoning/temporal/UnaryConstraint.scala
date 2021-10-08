package org.aiddl.common.scala.reasoning.temporal

import org.aiddl.core.scala.representation.Sym

object UnaryConstraint {
  val Release = Sym("release")
  val Deadline = Sym("deadline")
  val Duration = Sym("duration")
  val At = Sym("at")
}