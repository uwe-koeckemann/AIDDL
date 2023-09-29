package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.function.{Evaluator, Function, LazyFunction}
import org.aiddl.core.scala.representation.{EntRef, Term}

protected[function] class EvalAllRefsFunction(eval: Evaluator) extends Function with LazyFunction {
  def apply(x: Term): Term = {
    eval.followRefs = true
    val r = eval(x)
    eval.followRefs = false;
    r
  }
}
