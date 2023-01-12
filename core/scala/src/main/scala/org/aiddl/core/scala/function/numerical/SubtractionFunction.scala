package org.aiddl.core.scala.function.numerical

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{Term, Tuple}

protected[function] class SubtractionFunction extends Function {
  def apply(x: Term): Term = x match {
    case Tuple(args@_*) => args.tail.foldLeft(args.head)(_.asNum - _.asNum)
    case _ => x
  }
}
