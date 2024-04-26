package org.aiddl.common.scala.optimization.combinatorial.tsp

import org.aiddl.common.scala.math.graph.Graph
import org.aiddl.common.scala.optimization.{NumericalValueFunction, ValueFunction}
import org.aiddl.core.scala.representation.{ListTerm, Num}

class TspValueFunction(graph: Graph) extends NumericalValueFunction[ListTerm] {
  override val ordering: Ordering[Num] = (x: Num, y: Num) => x.compare(y)

  override def apply(x: ListTerm): Num =
    x.zip(x.tail).foldLeft(Num(0))( (c, edge) => {
      val (a, b) = edge
      c + this.graph.weight(a, b).get
    }) + this.graph.weight(x.reverse.head, x.head).get
}
