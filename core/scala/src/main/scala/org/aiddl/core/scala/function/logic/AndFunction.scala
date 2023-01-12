package org.aiddl.core.scala.function.logic

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{Bool, Term, Tuple}

protected[function] class AndFunction(eval: Evaluator) extends Function with LazyFunction {
  override def apply(x: Term): Term = x match {
    case Tuple(args@_*) => Bool(args.forall(x => eval(x) == Bool(true)))
    case _ => x
  }
}
