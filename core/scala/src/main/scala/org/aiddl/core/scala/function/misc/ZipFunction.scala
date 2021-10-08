package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.tools.ComboIterator


class ZipFunction extends Function with LazyFunction {
    def apply( x: Term ): Term = x match {
        case ListTerm(xs) => {
            val listArgs = xs.filter( e => e.isInstanceOf[ListTerm] )
            val minLen = listArgs.map( e => e.asList.size ).min
            ListTerm(for ( i <- 0 until minLen ) yield Tuple(listArgs.map( k => k(i) ): _*))
        }
        case _ => x
    }
}

