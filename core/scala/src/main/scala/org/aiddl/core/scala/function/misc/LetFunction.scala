package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{Substitution, Term, Tuple}

protected[function] class LetFunction(eval: Evaluator) extends Function with LazyFunction {
  override def apply(x: Term): Term = x match {
    case Tuple(vas, fTerm) => {
      val s = new Substitution()
      vas.asCol.foreach(x => s.add(x.asKvp.key, eval(x.asKvp.value)))
      eval(fTerm \ s)
    }
    case _ => x
  }
}
