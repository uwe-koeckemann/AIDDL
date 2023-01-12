package org.aiddl.common.scala.optimization.combinatorial.tsp

import scala.util.Random
import scala.collection.immutable.HashSet

import org.aiddl.core.scala.function.{Function, Initializable}
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.graph.Terms._
import org.aiddl.common.scala.math.graph.Graph
import org.aiddl.common.scala.math.graph.AdjacencyListGraph

import org.aiddl.core.scala.function.InterfaceImplementation
import org.aiddl.common.scala.search.TreeSearch

class TspSolver extends TreeSearch {
    val f_expand = new PathExpander()
    val f_minRemainder = new MinRemainder()
    var g: Graph = _

    override def init( g: Term ) = {
        this.g = new AdjacencyListGraph(g)
        f_expand.init(this.g)
        f_minRemainder.init(this.g)
        super.init(g)
    }

    override def expand: Option[Seq[Term]] = f_expand(choice)

    override def cost( a: List[Term] ): Option[Num] =
        Some(f_minRemainder(a) + a.foldLeft(Num(0))( (c, v) => c + g.weight(v.asKvp.key, v.asKvp.value).get))
}