package org.aiddl.core.scala.function.higher_order

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{ListTerm, SetTerm, Term, Tuple}

/** Map a function over a collection to create a new collection of function application results.
 *
 * @param c container used for evaluation
 */
protected class MapFunction(c: Container) extends Function with LazyFunction {
  private val eval = c.getFunctionOrPanic(D.EVAL).asInstanceOf[Evaluator]

  /** Apply function to each element in a collection term
   *
   * @param x expected to be a tuple composed of a term that evaluates to
   *          a function reference and a collection term to be mapped over
   * @return collection of results of application of function to input collection
   */
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
