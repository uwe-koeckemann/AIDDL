package org.aiddl.core.scala.function.higher_order

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{ListTerm, SetTerm, Term, Tuple}

/** Filter function evaluates first argument (to function reference and
 * applies resulting function as a predicate to filter the second argument.
 * @param c Container used to evaluate arguments
 */
protected class FilterFunction(c: Container) extends Function with LazyFunction {
  private val eval = c.getFunctionOrPanic(D.EVAL).asInstanceOf[Evaluator]

  /** Apply filter to collection term
   * @param x expected to be a tuple composed of a term that evaluates to
   *          a function reference and a collection term to be filtered
   * @return result of applying the filter to the collection
   */
  override def apply(x: Term): Term = x match {
    case Tuple(ft, ListTerm(l)) =>
      val f = eval(ft).asFunRef
      ListTerm(l.filter(x => f(eval(x)).boolVal))
    case Tuple(ft, SetTerm(s)) =>
      val f = eval(ft).asFunRef
      SetTerm(s.filter(x => f(eval(x)).boolVal))
    case _ => throw new IllegalArgumentException(
      s"Bad argument: $x. Need tuple (f c) where f evaluates to " +
      s"a function reference and c is a collection term.")
  }
}
