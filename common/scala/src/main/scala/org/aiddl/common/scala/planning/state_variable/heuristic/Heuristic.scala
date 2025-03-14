package org.aiddl.common.scala.planning.state_variable.heuristic

import org.aiddl.core.scala.representation.{Num, Term}
import org.aiddl.common.scala.search

trait Heuristic extends search.Heuristic[Term] {
  def apply(x: Term): Num
}
