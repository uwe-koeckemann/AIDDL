package org.aiddl.core.scala.function.logic

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{Bool, CollectionTerm, Term, Tuple}

protected[function] class ExistsFunction(eval: Evaluator) extends Function with LazyFunction {
  override def apply(x: Term): Term = x match {
    case Tuple(e, col, f) if col.isInstanceOf[CollectionTerm] => Bool(col.asCol.exists(x => e unify x match {
      case Some(s) => eval(f\s).boolVal
      case None => false }) )
    case _ => x
  }
}
