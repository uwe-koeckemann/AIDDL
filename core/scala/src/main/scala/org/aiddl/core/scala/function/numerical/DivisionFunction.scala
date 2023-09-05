package org.aiddl.core.scala.function.numerical

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{Num, Term, Tuple}

protected[function] class DivisionFunction extends Function {
    def apply( x: Term ): Term = x match {
            case Tuple(args @ _*) => args.tail.foldLeft(args.head)(_.asNum / _.asNum)
            case _ => throw new IllegalArgumentException(s"Bad argument: $x. Need tuple of numerical terms.")
    }
}