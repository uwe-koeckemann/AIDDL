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

import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_KeyVal

import scala.language.implicitConversions

class PathExpander extends Function with Initializable {
  var g: Graph = _

  def init( g: Term ) = this.g = AdjacencyListGraph(g)
  def init( g: Graph ) = this.g = g

  def apply( path: Term ): Term = {
    this(path.asList.toList) match {
      case Some(exp) => ListTerm(exp)
      case None => NIL
    }
  }

  def apply( path: Seq[Term] ): Option[Seq[Term]] = {
    if (path.size == g.nodes.size) {
      None
    } else if (path.isEmpty) {
        Some(List(g.nodes.head))
    } else {
      var nonOptions = path.toSet // .flatMap( e => List(e.key, e.value) ).toSet
      val last = path.head
      nonOptions = nonOptions + last
      Some(ListTerm(g.outNeighbors(last).withFilter(!nonOptions.contains(_))
        .map( n => n ).toSeq))
    }
  }
}