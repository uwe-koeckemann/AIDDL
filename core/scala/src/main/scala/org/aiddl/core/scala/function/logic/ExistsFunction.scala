package org.aiddl.core.scala.function.logic

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Evaluator, Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{Bool, CollectionTerm, Term, Tuple}

protected[function] class ExistsFunction(eval: Evaluator) extends Function with LazyFunction {
  override def apply(x: Term): Term = x match {
    case Tuple(e, col, f) if col.isInstanceOf[CollectionTerm] =>
      Bool(col.asCol.exists(x => e unify x match {
        case Some(s) => eval(f\s).boolVal
        case None => false }) )
    case _ => throw new IllegalArgumentException(s"Bad argument: $x. " +
      s"Need tuple (M C E), where M is a term, C a collection, and E is " +
      s"an expression that should evaluate to true or false.")
  }
}
