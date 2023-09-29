package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Evaluator, Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{ListTerm, Term}
import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_KeyVal

import scala.language.implicitConversions

protected[function] class CondFunction(eval: Evaluator) extends Function with LazyFunction {
  def apply(x: Term): Term = x match {
    case ListTerm(l) => eval(l.find(x => eval(x.key).boolVal).get.value)
    case _ => throw new IllegalArgumentException(s"Bad argument: $x. " +
      s"Need list of key-value pairs where the keys are conditions and the values are expressions.")
  }
}
