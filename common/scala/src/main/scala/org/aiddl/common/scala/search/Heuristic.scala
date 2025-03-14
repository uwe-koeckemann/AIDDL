package org.aiddl.common.scala.search

import org.aiddl.core.scala.representation.Num

trait Heuristic[N] {
  def apply(node: N): Num
}
