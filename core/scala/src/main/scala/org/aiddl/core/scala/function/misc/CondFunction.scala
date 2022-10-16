package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{ListTerm, Term, given_Conversion_Term_KeyVal}

protected[function] class CondFunction(eval: Evaluator) extends Function with LazyFunction {
  def apply(x: Term): Term = x match {
    case ListTerm(l) => eval(l.find(x => eval(x.key).boolVal).get.value)
    case _ => x
  }
}
