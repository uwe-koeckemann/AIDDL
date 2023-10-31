package org.aiddl.common.scala.optimization.combinatorial.tsp

import scala.util.Random
import scala.collection.immutable.HashSet

import org.aiddl.core.scala.function.{Function, Initializable, InterfaceImplementation}
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.graph.Terms._
import org.aiddl.common.scala.math.graph.Graph
import org.aiddl.common.scala.math.graph.AdjacencyListGraph

class MinRemainder extends Function with Initializable with InterfaceImplementation  {
  val interfaceUri = Sym("org.aiddl.common.optimization.combinatorial.tsp.heuristic")

  var g: Graph = _

  def init( g: Term ) = this.g = AdjacencyListGraph(g)
  def init( g: Graph ) = this.g = g

  def apply( path: Term ): Term = this.apply( path.asList.toList )
  def apply( path: List[Term] ): Num = {
    if path.length == g.nodes.length
    then Num(0)
    else {
      val dest = if path.isEmpty then Set.empty else path.tail.toSet
      val source = if path.isEmpty then Set.empty else path.reverse.tail.toSet
      val cheapest = g.nodes
        .filter(!dest.contains(_))
        .map(n1 => {
          val candidates = g.nodes
            .withFilter(n2 => n1 != n2 && !source.contains(n2))
            .map(n2 => g.weight(n2, n1).get)
          candidates.min
        })
      cheapest.reduce(_ + _)
    }
  }
}
