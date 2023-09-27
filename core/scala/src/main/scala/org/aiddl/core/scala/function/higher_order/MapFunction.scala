package org.aiddl.core.scala.function.higher_order

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Evaluator, Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{ListTerm, SetTerm, Term, Tuple}

/** Map a function over a collection to create a new collection of function application results.
 *
 * @param c container used for evaluation
 */
protected[function] class MapFunction(eval: Evaluator) extends Function with LazyFunction {
  /** Apply function to each element in a collection term
   *
   * @param x expected to be a tuple composed of a term that evaluates to
   *          a function reference and a collection term to be mapped over
   * @return collection of results of application of function to input collection
   */
  override def apply(x: Term): Term = x match {
    case Tuple(ft, colTerm) => {
      val f = eval(ft).asFunRef
      val collection = eval(colTerm)
      collection match {
        case ListTerm(l) =>
          ListTerm(l.map(f(_)))
        case SetTerm(s) =>
          SetTerm(s.map(f(_)))
        case _ =>
          throw new IllegalArgumentException(s"Cannot map over non collection term $x")
      }
    }
    case _ => throw new IllegalArgumentException(
      s"Bad argument: $x. Need tuple (f c) where f evaluates to " +
        s"a function reference and c is a collection term.")
  }
}
