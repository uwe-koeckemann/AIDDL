package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.*

protected[function] class ZipFunction extends Function with LazyFunction {
    def apply( x: Term ): Term = x match {
        case ListTerm(xs) => {
            val listArgs = xs.filter( e => e.isInstanceOf[ListTerm] )
            val minLen = listArgs.map( e => e.asList.size ).min
            ListTerm(for ( i <- 0 until minLen ) yield Tuple(listArgs.map( k => k(i) ): _*))
        }
        case _ => throw new IllegalArgumentException(s"Bad argument: $x. Expected list term.")
    }
}

