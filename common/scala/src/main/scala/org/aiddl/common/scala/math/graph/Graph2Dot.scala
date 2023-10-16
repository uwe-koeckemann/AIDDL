package org.aiddl.common.scala.math.graph

import scala.collection.mutable.HashMap
import scala.collection.mutable
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Configurable
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.math.graph.GraphType.*
import org.aiddl.common.scala.math.graph.Terms.*
import org.aiddl.common.scala.Common.NIL

import java.nio.charset.StandardCharsets
import java.nio.file.{Files, Paths}

class Graph2Dot(t: GraphType) extends Function {
  import Terms._
  import GraphType._

  private val edgeStr = t match { case Directed => "->"  case Undirected => "--" }
  private val graphTypeStr = t match { case Directed => "digraph"  case Undirected => "graph" }


  def apply( args: Term ): Term = {
    Str(extract(new AdjacencyListGraph(args)))
  }

  def graph2file( args: Term, fileName: String ) = {
    val graphStr = extract(new AdjacencyListGraph(args))
    Files.write(Paths.get(fileName), graphStr.getBytes(StandardCharsets.UTF_8))
  }

  def extract( g: Graph ): String = {
    val sb = new mutable.StringBuilder
    sb.append(s"$graphTypeStr {\n")

    val nodeMap = new HashMap[Term, Int]()
    var nextFree = 0

    g.nodes.foreach( u => {
      nextFree += 1
      nodeMap.put(u, nextFree)
      var label = u match {
        case Str(s) => s
        case t => t.toString
      }
      val additional = g.attributes(u) match {
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
          s append (atts.get(Sym("style")) match {
            case Some(shape) => s""", style="${shape}""""
            case None => ""
          })
          s append (atts.get(Sym("xlabel")) match {
            case Some(Str(xlabel)) => s""", xlabel="${xlabel}""""
            case Some(xlabel) => s""", xlabel="${xlabel}""""
            case None => ""
          })
          label = atts.get(Sym("label")) match {
            case Some(l) => if (l == NIL) "" else l match {
              case Str(s) => s
              case t => t.toString
            }
            case _ => label
          }
          s.toString
        }
        case None => ""
      }
      sb append s"""\tn$nextFree [label="$label"$additional];\n"""
    })
    sb.append("\n")

    g.edges.map(_.asList.toList).foreach( e => {
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
      val additional = g.edgeAttributes(v1, v2) match {
        case Some(atts) => {
          val s = new mutable.StringBuilder
          s append (atts.get(Sym("style")) match {
            case Some(shape) => s""", style="${shape}""""
            case None => ""
          })
          s.toString()
        }
        case None => ""
      }
      val config = if (eb.isEmpty) "" else " [label=" + eb.toString + s"$additional]"
      sb.append( s"""\tn${nodeMap(v1)} $edgeStr n${nodeMap(v2)}$config;\n""" )
    })
    sb.append("}")
    sb.toString
  }
}