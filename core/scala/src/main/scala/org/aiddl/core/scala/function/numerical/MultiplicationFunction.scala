package org.aiddl.core.scala.function.numerical

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{Num, Term, Tuple}

class MultiplicationFunction extends Function {
  def apply(x: Term): Term = x match {
    case Tuple(args@_*) => args.foldLeft(Num(1L): Term)(_.asNum * _.asNum)
    case _ => x
  }
}
