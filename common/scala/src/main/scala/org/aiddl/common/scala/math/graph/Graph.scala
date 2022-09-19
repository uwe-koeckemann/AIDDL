package org.aiddl.common.scala.math.graph

import scala.collection.mutable.HashMap
import scala.collection.mutable

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Configurable
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.Common.NIL

import org.aiddl.core.scala.representation.TermCollectionImplicits.term2CollectionTerm

trait Graph {
    def nodeCount: Int
    def edgeCount: Int
    def nodes: CollectionTerm
    def edges: CollectionTerm
    def inNeighbors(v: Term): CollectionTerm
    def outNeighbors(v: Term): CollectionTerm
    def incidentEdges( v: Term ): CollectionTerm
    def transpose: Graph

    def weight( u: Term, v: Term ): Option[Num] = None
    def label( u: Term, v: Term ): Option[Term] = None
    def attributes( u: Term ): Option[Term] = None
    def edgeAttributes( u: Term, v: Term ): Option[Term] = None
}


