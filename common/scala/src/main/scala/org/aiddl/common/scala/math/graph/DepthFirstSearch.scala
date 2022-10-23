package org.aiddl.common.scala.math.graph

import scala.collection.mutable.Map
import scala.collection.mutable.HashMap
import scala.collection.immutable.Set
import scala.collection.immutable.HashSet
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.InterfaceImplementation
import org.aiddl.core.scala.representation._
import org.aiddl.common.scala.Common._

class DepthFirstSearch extends Function with InterfaceImplementation {
    val interfaceUri = Sym("org.aiddl.common.math.graph.depth-first-search")
    enum Color:
        case White, Gray, Black

    import Color._
    var time = 0

    private val c = new HashMap[Term, Color]().withDefaultValue(White)
    val pi = new HashMap[Term, Term]().withDefaultValue(NIL) 
    val d = new HashMap[Term, Int]()
    val f = new HashMap[Term, Int]()
    var components: Set[Term] = _
    var g: Graph = _   

    def apply( g: Graph ): Unit = {
        this.g = g; time = 0; c.clear; pi.clear; d.clear; f.clear 
  
        components = g.nodes.withFilter(c(_) == White)
                            .map(u => {
                                SetTerm(visit(g, u))
                            }).toSet
    }

    private def visit( g: Graph, u: Term ): Set[Term] = {
        var visited = HashSet(u)
        time += 1
        d(u) = time
        c(u) = Gray
        g.outNeighbors(u).withFilter(c(_) == White).foreach( v =>
            pi(v) = u; visited = visited ++ visit(g, v))
        time += 1
        f(u) = time
        c(u) = Black
        visited
    }

    def apply( g: Term ): Term = {
        val graph = new AdjacencyListGraph(g)
        this(graph)
        outputTerm
    }

    def outputTerm: Term = {
        SetTerm(
            KeyVal(Sym("predecessor"), SetTerm(g.nodes.map(u => KeyVal(u, pi(u))).toSet)),
            KeyVal(Sym("distance"), SetTerm(g.nodes.map(u => KeyVal(u, Num(d(u)))).toSet)),
            KeyVal(Sym("finish-time"), SetTerm(g.nodes.map(u => KeyVal(u, Num(f(u)))).toSet)),
            KeyVal(Sym("components"), SetTerm(components))
        )
    }
}

