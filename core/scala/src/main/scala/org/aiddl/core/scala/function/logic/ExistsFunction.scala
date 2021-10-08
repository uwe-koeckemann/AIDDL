package org.aiddl.core.scala.function.logic

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.BoolImplicits.*
import org.aiddl.core.scala.representation.{Bool, CollectionTerm, Term, Tuple}


class ExistsFunction(c: Container) extends Function with LazyFunction {
  val eval = c.getFunctionOrPanic(D.EVAL).asInstanceOf[Evaluator]

  override def apply(x: Term): Term = x match {
      case Tuple(e, col, f) if col.isInstanceOf[CollectionTerm] => Bool(col.asCol.exists( x => (e unify x) match { case Some(s) => eval(f\s).asBool case None => Bool(false) }) )
      case _ => x
  }
}
