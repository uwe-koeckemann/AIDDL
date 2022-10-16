package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction}
import org.aiddl.core.scala.representation.{EntRef, Term}

protected[function] class EvalRefFunction(eval: Evaluator) extends Function with LazyFunction {
  def apply(x: Term): Term = {
    x match { case _: EntRef => eval(eval(x)) case _ => x }
  }
}
