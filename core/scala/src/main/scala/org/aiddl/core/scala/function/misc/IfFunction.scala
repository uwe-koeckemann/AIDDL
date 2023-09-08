package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{Term, Tuple}

protected[function] class IfFunction(eval: Evaluator) extends Function with LazyFunction {
  def apply(x: Term): Term = x match {
    case Tuple(cond, t, e) => if (eval(cond).asBool.v) eval(t) else eval(e)
    case _ => throw new IllegalArgumentException(s"Bad argument: $x. Expected tuple (cond a b) where a/b are expressions " +
      s"evaluated if cond evaluates to true/false.")
  }
}
