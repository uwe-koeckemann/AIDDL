package org.aiddl.common.scala.optimization.combinatorial.tsp

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.graph.GraphType.Undirected
import org.aiddl.common.scala.math.graph.{AdjacencyListGraph, Graph, Graph2Dot}
import org.aiddl.common.scala.math.graph.Terms.*
import org.aiddl.common.scala.search.GenericTreeSearch
import org.aiddl.core.scala.function.{Function, Initializable, InterfaceImplementation}
import org.aiddl.core.scala.representation.*

import scala.collection.immutable.HashSet
import scala.util.Random

import sys.process.*
import scala.language.postfixOps

class TspSolver extends GenericTreeSearch[Term, Term] with Initializable {
    val f_expand = new PathExpander()
    val f_minRemainder = new MinRemainder()
    var g: Graph = _
    var graphTerm: Term = _
    allowEarlyCostPruning = true

    override def init( g: Term ) = {
        this.graphTerm = g
        this.g = new AdjacencyListGraph(g)
        f_expand.init(this.g)
        f_minRemainder.init(this.g)
        super.reset
    }

    override def expand: Option[Seq[Term]] =
        f_expand(choice)

    override def cost( a: List[Term] ): Option[Num] =
        println(a)
        val cost =
            if a.length == 1
            then Num(0)
            else {
                val c = a.zip(a.tail).foldLeft(Num(0))((c, edge) => {
                    val (a, b) = edge
                    println(s"$a $b ${this.g.weight(a, b).get}")
                    c + this.g.weight(a, b).get
                }) + this.g.weight(a.reverse.head, a.head).get
                println(s"${a.reverse.head} ${a.head} ${this.g.weight(a.reverse.head, a.head).get}")
                c
            }

        Some(f_minRemainder(a) + cost) // a.foldLeft(Num(0))( (c, v) => c + g.weight(v.asKvp.key, v.asKvp.value).get))

    override val nil: Term = ListTerm.empty

    override def assembleSolution(choice: List[Term]): Option[Term] =
        Some(ListTerm(choice))
}