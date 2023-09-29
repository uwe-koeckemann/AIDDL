package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Evaluator, Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{Bool, ListTerm, Term, Tuple}

protected[function] class Match(eval: Evaluator) extends Function with LazyFunction {
  override def apply(x: Term): Term = {
    x match {
      case Tuple(p, t, f) => {
        val s = p unify t
        s match {
          case Some(s) => eval(f \ s)
          case None => Bool(false)
        }
      }
      case Tuple(p, l: ListTerm) => {
        val mCase: Option[Term] = l.find( c => c(0) unifiable p )
        mCase match {
          case Some(c) => {
            val s = (c(0) unify p).get // must work because of find above
            eval(c(1) \ s)
          }
          case None => throw new IllegalArgumentException(s"Match error: ${x}")
        }
      }
      case _ => throw new IllegalArgumentException(s"Bad argument: $x. Expected tuple with two or three arguments.")    }
  }
}
