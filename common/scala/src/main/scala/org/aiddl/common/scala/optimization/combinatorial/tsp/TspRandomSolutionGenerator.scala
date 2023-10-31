package org.aiddl.common.scala.optimization.combinatorial.tsp

import org.aiddl.common.scala.math.graph.Graph
import org.aiddl.common.scala.search.local.SolutionProvider
import org.aiddl.core.scala.representation.{KeyVal, ListTerm, Term}

import scala.util.Random

class TspRandomSolutionGenerator(graph: Graph, random: Random = new Random()) extends SolutionProvider[ListTerm] {
  def apply(): ListTerm = {
    val source: List[Term] = graph.nodes.toList
    val shuffled = random.shuffle(source)
    //val path = shuffled.zip(shuffled.tail).map((a, b) => KeyVal(a, b)).appended(KeyVal(shuffled.reverse.head, shuffled.head))
    ListTerm(shuffled)
  }
}
