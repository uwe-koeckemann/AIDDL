package org.aiddl.common.scala.planning.state_variable.data

import scala.collection.immutable.HashSet
import scala.collection.mutable.HashMap
import scala.collection.mutable

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.InterfaceImplementation
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.math.graph.{Graph, GraphTools, AdjacencyListGraph, StronglyConnectedComponentExtractor}
import org.aiddl.common.scala.planning.PlanningTerm.{Preconditions, Effects}

import Term.given_Conversion_Term_KeyVal
import scala.language.implicitConversions

class CausalGraphCreator extends Function with InterfaceImplementation {
    val interfaceUri = Sym("org.aiddl.common.planning.state-variable.data.causal-graph-creator");
    val fScc = new StronglyConnectedComponentExtractor

    def apply( os: Term ): Term = {
        var nodes = new HashSet[Term]
        var edges = new HashSet[Term]

        os.asCol.foreach( o => {
            o(Effects).asCol.foreach( e1 => {
                nodes = nodes + e1.key
                o(Effects).asCol.withFilter(e2 => e1.key != e2.key).foreach( e2 => {
                    nodes = nodes + e2.key
                    edges = edges + Tuple(e1.key, e2.key) })
                o(Preconditions).asCol.withFilter(p => e1.key != p.key).foreach( p => {
                    nodes = nodes + p.key
                    edges = edges + Tuple(p.key, e1.key) }) }) })

        val g = AdjacencyListGraph(SetTerm(nodes), SetTerm(edges))
        val sccs = fScc(g)
        val cycleFreeEdges: Set[Term] = sccs.asCol.flatMap( scc => {
            val order = totalOrder(scc.asCol, g, HashSet.empty, scc.asSet.set)  
          
            scc.asCol.flatMap( u =>
                g.outNeighbors(u).withFilter( v => {
                    !scc.asCol.contains(v) || order.indexOf(u) < order.indexOf(v) 
                } )
                .map(v => Tuple(u, v)) )
        }).toSet

        GraphTools.assembleGraph(SetTerm(nodes), SetTerm(cycleFreeEdges))
    }

    //private def totalOrder( scc: CollectionTerm, g: Graph ): Seq[Term] = 
    //   scc.asCol.toList.sortBy( u => g.inNeighbors(u).count( v => scc.contains(v) ) )

    private def totalOrder( scc: CollectionTerm, g: Graph, ignored: HashSet[Term], todo: Set[Term] ): List[Term] = {
        if (todo.isEmpty) Nil else {
            val argMin = todo.minBy( u => g.inNeighbors(u).count( v => scc.contains(v) && !ignored.contains(Tuple(u, v)) ) )
            argMin :: totalOrder(scc, g, ignored + g.incidentEdges(argMin), todo - argMin )
        }
    }
}