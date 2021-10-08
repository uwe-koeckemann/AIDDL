package org.aiddl.core.scala.function.logic

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{Bool, Term, Tuple}
import org.aiddl.core.scala.representation.BoolImplicits.*

class ForallFunction(c: Container) extends Function with LazyFunction {
  val eval = c.getFunctionOrPanic(D.EVAL).asInstanceOf[Evaluator]

  override def apply(x: Term): Term = x match {
    case Tuple(e, col, f) => Bool(eval(col).asCol.forall(x => (e unify x) match {
      case Some(s) => eval(f \ s).asBool
      case None => Bool(false)
    }))
    case _ => x
  }
}
