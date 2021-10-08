package org.aiddl.common.scala.math.graph

import org.aiddl.core.scala.function.Function

import scala.collection.mutable.HashMap

import org.aiddl.core.scala.representation._
import org.aiddl.core.scala.function.InterfaceImplementation

import org.aiddl.common.scala.Common._
import org.aiddl.common.scala.math.graph.Terms._

import org.aiddl.core.scala.representation.TermImplicits._

class BellmanFord extends Function with InterfaceImplementation {
    val interfaceUri = Sym("org.aiddl.common.math.graph.single-source-shortest-path")

    val dist = new HashMap[Term, Num]().withDefaultValue(InfPos())
    val pi = new HashMap[Term, Term]().withDefaultValue(NIL)
    var g: Graph = _
    var hasNegativeCycle = false

    def apply( args: Term ): Term = args match {
        case Tuple(gt, w, s) => {
            g = new AdjacencyListGraph(gt)
            this(g, s, w)
            outputTerm
        }
        case _ => ???
    } 

    def apply( g: Graph, s: Term, w: Function ): Boolean = {
        dist.clear; pi.clear
        dist.put(s, Num(0))
        (1 to g.nodeCount).foreach( _ =>
            g.edges.foreach( e => e match { case Tuple(u, v) => {
                if (dist(v) > dist(u) + w(e)) { dist.put(v, dist(u) + w(e)); pi.put(v, u) }
            } case _ => } ))

        hasNegativeCycle = g.edges.exists(e => e match { case Tuple(v1, v2) => dist(v2) > dist(v1) + w(e) case _ => false })
        hasNegativeCycle
    }

    def outputTerm: Term =  
        if ( hasNegativeCycle ) {
            SetTerm(Sym("distance") :: NIL, Sym("predecessor") :: NIL)
        } else {
            SetTerm(
                Sym("distance") :: SetTerm(g.nodes.map( v => KeyVal(v, dist(v))).toSet),
                Sym("predecessor") :: SetTerm( g.nodes.map( v => KeyVal(v, pi(v))).toSet)) 
        }
}

