package org.aiddl.common.scala.search.local

trait Neighborhood[S] {
  def apply(solution: S): IterableOnce[S]
}
