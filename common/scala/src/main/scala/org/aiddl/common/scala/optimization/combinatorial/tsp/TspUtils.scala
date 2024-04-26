package org.aiddl.common.scala.optimization.combinatorial.tsp

import org.aiddl.common.scala.math.graph.Graph2Dot
import org.aiddl.common.scala.math.graph.GraphType.Undirected
import org.aiddl.common.scala.math.graph.Terms.{Edges, Weights}
import org.aiddl.core.scala.representation.*

object TspUtils {

  def pathGraphToFile(graph: Term, filename: String, compile: Boolean = true): Unit = {
    val graph2Dot = new Graph2Dot(Undirected)
    graph2Dot.graph2file(graph, s"$filename.dot")
    if ( compile ) {
      Graph2Dot.compileWithPos(filename)
    }
  }

  def pathGraph(graph: Term, path: List[Term]): Term = {
    val edges = SetTerm(path.map(kvp => SetTerm(kvp.asKvp.key, kvp.asKvp.value)).toSet)
    graph.asCol.put(KeyVal(Edges, edges)).put(KeyVal(Weights, SetTerm.empty))
  }
}
