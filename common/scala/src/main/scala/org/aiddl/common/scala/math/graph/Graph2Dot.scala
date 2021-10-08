package org.aiddl.common.scala.math.graph

import scala.collection.mutable.HashMap
import scala.collection.mutable

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Configurable
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.graph.GraphType._
import org.aiddl.common.scala.math.graph.Terms._
import org.aiddl.common.scala.Common.NIL

import org.aiddl.core.scala.representation.TermCollectionImplicits.term2CollectionTerm

class Graph2Dot(t: GraphType) extends Function {
  import Terms._
  import GraphType._

  private var edgeStr = t match { case Directed => "->"  case Undirected => "--" }

  def apply( args: Term ): Term = {
    Str(extract(new AdjacencyListGraph(args)))
  }

  def extract( g: Graph ): String = {
    val sb = new mutable.StringBuilder
    sb.append("graph {\n")

    val nodeMap = new HashMap[Term, Int]()
    var nextFree = 0

    g.nodes.foreach( u => {
      nextFree += 1
      nodeMap.put(u, nextFree)
      var label = u.toString
      var additional = g.attributes(u) match {
        case Some(atts) => {
          var s = new mutable.StringBuilder
          s append (atts.get(Sym("pos")) match {
            case Some(pos) => s""", pos="${pos(0)},${pos(1)}!""""
            case None => ""
          })
          s append (atts.get(Sym("shape")) match {
            case Some(shape) => s""", shape="${shape}""""
            case None => ""
          })
          label = atts.get(Sym("label")) match {
            case Some(l) => if (l == NIL) "" else l.toString
            case _ => label
          }
          s.toString
        }
        case None => ""
      }
      sb append s"""\tn$nextFree [label="$label"$additional];\n"""
    })
    sb.append("\n")

    g.edges.map(_.toList).foreach( e => {
      val v1 = e(0)
      val v2 = e(1)
      val eb = new mutable.StringBuilder
      g.weight(v1, v2) match {
        case Some(w) => eb.append(w.toString)
        case None => {}
      }
      g.label(v1, v2) match {
        case Some(l) => {
          if (!eb.isEmpty) eb.append(", ")
          eb.append(l.toString)
        }
        case None => {}
      }
      val config = if (sb.isEmpty) "" else " [" + sb.toString + "]"
      sb.append( s"""\tn${nodeMap(v1)} $edgeStr n${nodeMap(v2)}$config;\n""" )
    })
    sb.append("}")
    sb.toString
  }
}