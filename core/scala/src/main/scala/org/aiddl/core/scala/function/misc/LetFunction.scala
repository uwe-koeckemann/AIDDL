package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Evaluator, Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{Substitution, Term, Tuple}

protected[function] class LetFunction(eval: Evaluator) extends Function with LazyFunction {
  override def apply(x: Term): Term = x match {
    case Tuple(vas, fTerm) => {
      val s = new Substitution()
      vas.asCol.foreach(x => s.add(x.asKvp.key, eval(x.asKvp.value)))
      eval(fTerm \ s)
    }
    case _ => throw new IllegalArgumentException(s"Bad argument: $x. Expected tuple (X E) where X is a " +
      "collection of key-value pairs whose values are evaluated to create a substitution and E is an expression" +
      "that is evaluated after the substitution is applied.")
  }
}
