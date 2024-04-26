package org.aiddl.common.scala.optimization.combinatorial.tsp

import org.aiddl.common.scala.search.local.Neighborhood
import org.aiddl.core.scala.representation.{KeyVal, ListTerm}

class Tsp2OptNeighborhood extends Neighborhood[ListTerm] {
  override def apply(solution: ListTerm): IterableOnce[ListTerm] = {
    var neighborList: List[ListTerm] = Nil
    for {
      i <- 0 until solution.length
      j <- 0 until solution.length
      if i != j
    } {
      val swappedSolution = ListTerm(solution.updated(i, solution(j)).updated(j, solution(i)))
      neighborList = swappedSolution :: neighborList
    }
    neighborList
  }
}
