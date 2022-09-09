package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{ListTerm, Term}
import org.aiddl.core.scala.representation.given_Conversion_Term_KeyVal

class CondFunction(c: Container) extends Function with LazyFunction {
  val eval = c.getFunctionOrPanic(D.EVAL).asInstanceOf[Evaluator]

  def apply(x: Term): Term = x match {
    case ListTerm(l) => eval(l.find(x => eval(x.key).boolVal).get.value)
    case _ => x
  }
}
