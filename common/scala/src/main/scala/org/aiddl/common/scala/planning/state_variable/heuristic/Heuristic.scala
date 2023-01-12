package org.aiddl.common.scala.planning.state_variable.heuristic

import org.aiddl.core.scala.representation.{Num, Term}

trait Heuristic {
  def apply(x: Term): Num
}
