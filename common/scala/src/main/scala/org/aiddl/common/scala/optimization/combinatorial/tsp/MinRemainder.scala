package org.aiddl.common.scala.optimization.combinatorial.tsp

import scala.util.Random
import scala.collection.immutable.HashSet

import org.aiddl.core.scala.function.{Function, Initializable, InterfaceImplementation}
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.graph.Terms._
import org.aiddl.common.scala.math.graph.Graph
import org.aiddl.common.scala.math.graph.AdjacencyListGraph

import org.aiddl.common.scala.search.TreeSearch

class MinRemainder extends Function with Initializable with InterfaceImplementation  {
  val interfaceUri = Sym("org.aiddl.common.optimization.combinatorial.tsp.heuristic")

  var g: Graph = _

  def init( g: Term ) = this.g = AdjacencyListGraph(g)
  def init( g: Graph ) = this.g = g

  def apply( path: Term ): Term = this.apply( path.asList.toList )
  def apply( path: List[Term] ): Num = {
    val (in, out) = path.foldLeft((Set.empty, Set.empty))( (c, e) => (c._1 + e.asKvp.key, c._2 + e.asKvp.value) )
    g.nodes.filter(!out.contains(_))
      .foldLeft(Num(0))( (c, n1) => c + g.nodes.withFilter( n2 => n1 != n2 && !in.contains(n2))
        .map( n2 => g.weight(n1, n2).get ).min  )
  }
}
