package org.aiddl.common.scala.reasoning.temporal

import org.aiddl.core.scala.representation.Sym

object AllenConstraint {
  val Equals = Sym("equals")
  val Before = Sym("before")
  val After = Sym("after")
  val Meets = Sym("meets")
  val MetBy = Sym("met-by")
  val Starts = Sym("starts")
  val StartedBy = Sym("started-by")
  val Finishes = Sym("finishes")
  val FinishedBy = Sym("finished-by")
  val During = Sym("during")
  val Contains = Sym("contains")
  val Overlaps = Sym("overlaps")
  val OverlappedBy = Sym("overlapped-by")
}