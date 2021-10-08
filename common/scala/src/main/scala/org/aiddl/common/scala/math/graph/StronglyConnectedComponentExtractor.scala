package org.aiddl.common.scala.math.graph

import org.aiddl.core.scala.function.Function

import scala.collection.mutable.HashMap

import org.aiddl.core.scala.representation._
import org.aiddl.core.scala.function.InterfaceImplementation

import org.aiddl.common.scala.Common._
import org.aiddl.common.scala.math.graph.Terms._

import org.aiddl.core.scala.representation.TermCollectionImplicits.seq2Tuple
import org.aiddl.core.scala.representation.TermCollectionImplicits.set2Term

class StronglyConnectedComponentExtractor extends Function with InterfaceImplementation {
    val interfaceUri = Sym("org.aiddl.common.math.graph.compute-scc")

    private val fDFS = new DepthFirstSearch

    def apply( g: Term ): Term = this(new AdjacencyListGraph(g))

    def apply( g: Graph ): Term = {
        fDFS(g) 
        val gtbf = new AdjacencyListGraph(Tuple(
            Nodes :: ListTerm(g.nodes.asList.sortBy( n => -fDFS.f(n) ) ), 
            Edges :: SetTerm(g.edges.map{ case Tuple(v, u) => Tuple(u, v) case _ => ??? }.toSet)))
        fDFS(gtbf)
        SetTerm(fDFS.components)
    }     
}