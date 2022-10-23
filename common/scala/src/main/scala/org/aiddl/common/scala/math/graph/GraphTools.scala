package org.aiddl.common.scala.math.graph

import scala.collection.mutable.HashMap
import scala.collection.mutable

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Configurable
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.graph.Terms._
import org.aiddl.common.scala.math.graph.GraphType._

object GraphTools {
  def assembleGraph(n: CollectionTerm, e: CollectionTerm): Tuple = Tuple( Nodes :: n, Edges :: e )

  def unpackEdge(x: Term): (GraphType,Term,Term) = {
    val v1 = x(0)
    val v2 = x(1)
    (if (x.isInstanceOf[Tuple]) Directed else Undirected, v1, v2)
  }
}