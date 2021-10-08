package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{Bool, Term, Tuple}

class Match(c: Container) extends Function with LazyFunction {
  val eval = c.getFunctionOrPanic(D.EVAL).asInstanceOf[Evaluator]

  override def apply(x: Term): Term = x match {
    case Tuple(p, t, f) => {
      val s = p unify t
      s match {
        case Some(s) => eval(f \ s)
        case None => Bool(false)
      }
    }
    case _ => x
  }
}
