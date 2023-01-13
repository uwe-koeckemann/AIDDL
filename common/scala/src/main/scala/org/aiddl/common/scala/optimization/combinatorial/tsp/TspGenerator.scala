package org.aiddl.common.scala.optimization.combinatorial.tsp

import scala.util.Random
import scala.collection.immutable.HashSet

import org.aiddl.core.scala.function.{Function, Initializable}
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.graph.Terms._
import org.aiddl.common.scala.math.graph.Graph
import org.aiddl.common.scala.math.graph.AdjacencyListGraph

import org.aiddl.core.scala.function.InterfaceImplementation
import org.aiddl.common.scala.search.TreeSearch

import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_Num

import scala.language.implicitConversions

class TspGenerator extends Function {

  def apply( args: Term ): Term = ???

  def apply( n: Int, xMax: Int, yMax: Int ): Term = {
    val r = new Random
    val nodes: Set[Term] = (1 to n).map( i => Sym(s"n$i") ).toSet
    val coordinates = SetTerm((1 to n).map( i =>
      KeyVal(Sym(s"n$i"), SetTerm(
        KeyVal(Sym("pos"), Tuple(Num(r.nextInt(xMax)), Num(r.nextInt(yMax))))))).toSet)

    var edges: HashSet[Term] = HashSet.empty
    var weights: HashSet[Term] = HashSet.empty
    nodes.foreach( n1 => nodes.foreach( n2 => {
      if ( n1 != n2 ) {
        val x1 = coordinates(n1)(Sym("pos"))(0).intoDouble
        val y1 = coordinates(n1)(Sym("pos"))(1).intoDouble
        val x2 = coordinates(n2)(Sym("pos"))(0).intoDouble
        val y2 = coordinates(n2)(Sym("pos"))(1).intoDouble

        val dist = Math.sqrt(Math.pow(x1-x2, 2.0) + Math.pow(y1-y2, 2.0))
        val e = SetTerm(n1, n2)
        edges = edges + e
        weights = weights + KeyVal(e, Num(dist))
      }
    }))

    Tuple(
      KeyVal(Nodes, SetTerm(nodes)),
      KeyVal(Edges, SetTerm(edges)),
      KeyVal(Weights, SetTerm(weights)),
      KeyVal(Attributes, SetTerm(coordinates))
    )

  }

}