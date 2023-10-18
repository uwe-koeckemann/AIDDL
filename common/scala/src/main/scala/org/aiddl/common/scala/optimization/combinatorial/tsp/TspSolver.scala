package org.aiddl.common.scala.optimization.combinatorial.tsp

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.graph.{AdjacencyListGraph, Graph}
import org.aiddl.common.scala.math.graph.Terms.*
import org.aiddl.common.scala.search.{GenericTreeSearch, TreeSearch}
import org.aiddl.core.scala.function.{Function, Initializable, InterfaceImplementation}
import org.aiddl.core.scala.representation.*

import scala.collection.immutable.HashSet
import scala.util.Random

class TspSolver extends GenericTreeSearch[Term, Term] with Initializable {
    val f_expand = new PathExpander()
    val f_minRemainder = new MinRemainder()
    var g: Graph = _

    override def init( g: Term ) = {
        this.g = new AdjacencyListGraph(g)
        f_expand.init(this.g)
        f_minRemainder.init(this.g)
        super.reset
    }

    override def expand: Option[Seq[Term]] =
        f_expand(choice)

    override def cost( a: List[Term] ): Option[Num] =
        Some(f_minRemainder(a) + a.foldLeft(Num(0))( (c, v) => c + g.weight(v.asKvp.key, v.asKvp.value).get))

    override val nil: Term = ListTerm.empty

    override def assembleSolution(choice: List[Term]): Option[Term] =
        Some(ListTerm(choice))
}