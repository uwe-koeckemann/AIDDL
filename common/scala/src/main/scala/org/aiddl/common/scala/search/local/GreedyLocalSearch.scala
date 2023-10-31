package org.aiddl.common.scala.search.local

import org.aiddl.common.scala.optimization.ValueFunction
import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.representation.Num

trait GreedyLocalSearch[S, N] extends Verbose {

  val solutionProvider: SolutionProvider[S]
  val valueFunction: ValueFunction[S, N]
  val neighborhood: Neighborhood[S]

  /**
   * Optional target value. If any solution with equal or better value is found,
   * search will terminate immediately.
   */
  val targetValue: Option[N] = None

  /**
   * How often the search is restarted
   */
  var numTries = 1

  def search: S = {
    var overallSolution: S = solutionProvider()
    var i = 0
    var targetReached = false

    while ( i < numTries && !targetReached ) {
      i += 1
      var solution = solutionProvider()
      var localOptimum = false
      while (!localOptimum && !targetReached) {
        val bestNeighbor = neighborhood(solution).iterator.min(this.valueFunction)
        if valueFunction.gteq(bestNeighbor, solution)
        then solution = bestNeighbor
        else localOptimum = true

        targetValue.foreach(value => {
          if valueFunction.ordering.lteq(valueFunction(solution), value)
          then targetReached = true
        })
      }

      if valueFunction.lteq(solution, overallSolution)
      then overallSolution = solution
      logger.info(s"Iteration $i score: ${valueFunction(solution)} solution: $solution")
    }
    overallSolution
  }
}
