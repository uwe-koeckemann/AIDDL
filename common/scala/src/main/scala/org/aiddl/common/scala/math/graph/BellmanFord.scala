package org.aiddl.common.scala.math.graph

import org.aiddl.core.scala.function.Function

import scala.collection.mutable.HashMap
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.function.InterfaceImplementation
import org.aiddl.common.scala.Common.*
import org.aiddl.common.scala.math.graph.Terms.*

import scala.collection.mutable

class BellmanFord extends Function with InterfaceImplementation {
    val interfaceUri: Sym = Sym("org.aiddl.common.math.graph.single-source-shortest-path")

    val dist: mutable.Map[Term, Num] = new mutable.HashMap[Term, Num]().withDefaultValue(InfPos())
    val pi: mutable.Map[Term, Term] = new mutable.HashMap[Term, Term]().withDefaultValue(NIL)
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

    private def update(u: Term, v: Term, e: Term, w: Function): Unit = {
        if (dist(v) > dist(u) + w(e).asNum) {
            dist.put(v, dist(u) + w(e).asNum)
            pi.put(v, u)
        }
    }

    def apply( g: Graph, s: Term, w: Function ): Boolean = {
        dist.clear; pi.clear
        dist.put(s, Num(0))
        (1 to g.nodeCount).foreach( _ =>
            g.edges.foreach( e => e match {
                case Tuple(u, v) => update(u, v, e, w)
                case e: SetTerm => {
                    val u = e.head
                    val v = e.tail.head
                    update(u, v, e, w)
                    update(v, u, e, w)
                }
                case _ => {}
            } ))

        hasNegativeCycle = g.edges.exists(e => e match { case Tuple(v1, v2) => dist(v2) > dist(v1) + w(e).asNum case _ => false })
        hasNegativeCycle
    }

    def outputTerm: Term =  
        if ( hasNegativeCycle ) {
            SetTerm(KeyVal(Sym("distance"), NIL), KeyVal(Sym("predecessor"), NIL))
        } else {
            SetTerm(
                KeyVal(Sym("distance"), SetTerm(g.nodes.map( v => KeyVal(v, dist(v))).toSet)),
                KeyVal(Sym("predecessor"), SetTerm( g.nodes.map( v => KeyVal(v, pi(v))).toSet)))
        }
}

