package org.aiddl.core.scala.function.logic

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{Bool, Term, Tuple}

protected[function] class ForallFunction(eval: Evaluator) extends Function with LazyFunction {
  override def apply(x: Term): Term = x match {
    case Tuple(e, col, f) => Bool(eval(col).asCol.forall(x => e unify x match {
      case Some(s) => eval(f \ s).boolVal
      case None => false
    }))
    case _ => throw new IllegalArgumentException(s"Bad argument: $x. " +
      s"Need tuple (M C E), where M is a term, C a collection, and E is " +
      s"an expression that should evaluate to true or false.")
  }
}
