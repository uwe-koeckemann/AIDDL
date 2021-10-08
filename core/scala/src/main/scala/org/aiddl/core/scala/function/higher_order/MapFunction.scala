package org.aiddl.core.scala.function.higher_order

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{ListTerm, SetTerm, Term, Tuple}

class MapFunction(c: Container) extends Function with LazyFunction {
  val eval = c.getFunctionOrPanic(D.EVAL).asInstanceOf[Evaluator]

  override def apply(x: Term): Term = x match {
    case Tuple(ft, ListTerm(l)) => {
      val f = eval(ft).asFunRef; ListTerm(l.map(f(_)))
    }
    case Tuple(ft, SetTerm(s)) => {
      val f = eval(ft).asFunRef; SetTerm(s.map(f(_)))
    }
    case _ => x
  }
}
