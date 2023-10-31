package org.aiddl.common.scala.optimization.combinatorial.tsp

import org.aiddl.common.scala.math.graph.Graph
import org.aiddl.common.scala.optimization.ValueFunction
import org.aiddl.common.scala.search.local.{GreedyLocalSearch, Neighborhood, SolutionProvider}
import org.aiddl.core.scala.representation.*

class TspGreedyLocalSearch(start: ListTerm, graph: Graph) extends GreedyLocalSearch[ListTerm, Num] {
  override val solutionProvider: SolutionProvider[ListTerm] = new TspRandomSolutionGenerator(graph)
  override val valueFunction: ValueFunction[ListTerm, Num] = new TspValueFunction(graph)
  override val neighborhood: Neighborhood[ListTerm] = new Tsp2OptNeighborhood
}
