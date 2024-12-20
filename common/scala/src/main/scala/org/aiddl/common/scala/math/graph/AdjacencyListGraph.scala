package org.aiddl.common.scala.math.graph

import scala.collection.mutable.HashMap
import scala.collection.mutable

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Configurable
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.graph.Graph
import org.aiddl.common.scala.math.graph.GraphTools
import org.aiddl.common.scala.math.graph.GraphType._
import org.aiddl.common.scala.math.graph.Terms._

object AdjacencyListGraph {
  def apply(n: CollectionTerm, e: CollectionTerm): AdjacencyListGraph =
    new AdjacencyListGraph( Tuple(KeyVal(Nodes, n), KeyVal(Edges, e) ) )

  def apply(g: Term): AdjacencyListGraph = new AdjacencyListGraph( g )
}

class AdjacencyListGraph(g: Term) extends Graph {
  private lazy val adjListOut = {
    val r = new mutable.HashMap[Term, List[Term]]().withDefaultValue(Nil)
    edges.foreach( x => GraphTools.unpackEdge(x) match {
      case (Directed, v1, v2) => r.put(v1, v2 :: r(v1))
      case (Undirected, v1, v2) => { r.put(v1, v2 :: r(v1)); r.put(v2, v1 :: r(v2)) }
    }); r}

  private lazy val adjListIn = {
    val r = new mutable.HashMap[Term, List[Term]]().withDefaultValue(Nil)
    edges.foreach( x => GraphTools.unpackEdge(x) match {
      case (Directed, v1, v2) => r.put(v2, v1 :: r(v2))
      case (Undirected, v1, v2) => { r.put(v1, v2 :: r(v1)); r.put(v2, v1 :: r(v2)) }
    }); r}

  def nodeCount: Int = g(Nodes).asCol.size
  def edgeCount: Int = g(Edges).asCol.size
  def nodes: CollectionTerm = g(Nodes).asCol
  def edges: CollectionTerm = g(Edges).asCol
  def inNeighbors(v: Term): CollectionTerm = ListTerm(adjListIn.getOrElse(v, Nil))
  def outNeighbors(v: Term): CollectionTerm = ListTerm(adjListOut.getOrElse(v, Nil))
  def incidentEdges( v: Term ): CollectionTerm =
    SetTerm(edges.collect( e => GraphTools.unpackEdge(e) match {
      case (_, v1, v2)  if (v1 == v) || (v2 == v) => e } ).toSet)

  override def weight( u: Term, v: Term ): Option[Num] = {
    g.get(Weights) match {
      case Some(w) =>  {
        w.get(Tuple(u, v)) match {
          case Some(weight) => Some(weight.asNum)
          case None => w.get(SetTerm(u, v)) match {
            case Some(weight) => Some(weight.asNum)
            case None => None }
        }
      }
      case None => None
    }
  }
  override def label( u: Term, v: Term ): Option[Term] = {
    g.get(Labels) match {
      case Some(l) => l.get(Tuple(u, v)) match {
        case Some(label) => Some(label)
        case None => l.get(SetTerm(u, v))
      }
      case None => None
    }
  }

  def transpose: Graph = {
    val edgesTrans = SetTerm(
      g(Edges).asCol.map( e => GraphTools.unpackEdge(e) match {
        case (Directed, u, v) => Tuple(u, v)
        case (Undirected, u, v) => e
      } ).toSet)
    AdjacencyListGraph(Tuple(KeyVal(Nodes, g(Nodes)), KeyVal(Edges, edgesTrans)))
  }

  override def attributes(u: Term): Option[Term] = this.g.get(Attributes).flatMap(atts => atts.asCol.get(u))
  override def edgeAttributes(u: Term, v: Term): Option[Term] = {
    this.g.get(EdgeAttributes).flatMap(atts =>
      atts.get(Tuple(u, v)) match {
        case Some(edgeAtts) => Some(edgeAtts)
        case None => atts.get(SetTerm(u, v))
    } )
  }

  override def toString: String = g.toString
}