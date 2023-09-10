package org.aiddl.core.scala.function.higher_order

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Evaluator, Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{ListTerm, SetTerm, Term, Tuple}

/** Filter function evaluates first argument (to function reference and
 * applies resulting function as a predicate to filter the second argument.
 * @param c Container used to evaluate arguments
 */
protected[function] class FilterFunction(eval: Evaluator) extends Function with LazyFunction {
  /** Apply filter to collection term
   * @param x expected to be a tuple composed of a term that evaluates to
   *          a function reference and a collection term to be filtered
   * @return result of applying the filter to the collection
   */
  override def apply(x: Term): Term = x match {
    case Tuple(ft, colTerm) => {
      val f = eval(ft).asFunRef
      val collection = eval(colTerm)
      collection match {
        case ListTerm(l) =>
          ListTerm(l.filter(f(_).boolVal).toList)
        case SetTerm(s) =>
          SetTerm(s.filter(f(_).boolVal).toSet)
      }
    }
    case _ => throw new IllegalArgumentException(
      s"Bad argument: $x. Need tuple (f c) where f evaluates to " +
        s"a function reference and c is a collection term.")
  }
}
