package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{Term, Tuple}

class IfFunction(c: Container) extends Function with LazyFunction {
  val eval = c.getFunctionOrPanic(D.EVAL).asInstanceOf[Evaluator]

  def apply(x: Term): Term = x match {
    case Tuple(cond, t, e) => if (eval(cond).asBool.v) eval(t) else eval(e)
    case _ => ???
  }
}
