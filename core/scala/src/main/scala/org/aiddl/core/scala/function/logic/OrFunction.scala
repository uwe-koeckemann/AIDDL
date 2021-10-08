package org.aiddl.core.scala.function.logic

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{Bool, Term, Tuple}
import org.aiddl.core.scala.representation.BoolImplicits.*

class OrFunction(c: Container) extends Function with LazyFunction {
  val eval = c.getFunctionOrPanic(D.EVAL).asInstanceOf[Evaluator]

  override def apply(x: Term): Term = x match {
    case Tuple(args@_*) => Bool(args.exists(x => eval(x) == Bool(true)))
    case _ => x
  }
}
