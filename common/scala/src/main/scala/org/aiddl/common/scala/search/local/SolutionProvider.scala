package org.aiddl.common.scala.search.local

trait SolutionProvider[S] {
  def apply(): S
}
