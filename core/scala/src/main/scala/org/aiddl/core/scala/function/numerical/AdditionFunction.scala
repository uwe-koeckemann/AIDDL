package org.aiddl.core.scala.function.numerical

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{Num, Term, Tuple}

protected[function] class AdditionFunction extends Function {
  def apply(x: Term): Term = x match {
    case Tuple(args@_*) => args.reduce(_.asNum + _.asNum)
    case _ => throw new IllegalArgumentException(s"Bad argument: $x. Need tuple of numerical terms.")
  }
}
