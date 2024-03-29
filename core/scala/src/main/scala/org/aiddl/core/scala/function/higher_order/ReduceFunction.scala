package org.aiddl.core.scala.function.higher_order

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Evaluator, Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.*

/** Reduce a collection to a single term by applying a function to an accumulated values and the next value of the
 * collection. Optionally an initial accumulated value can be set with the symbolic key 'initial-value'. Otherwise,
 * the first value of the collection will be used as the initial value.
 *
 * Example (assuming name space for simplicity):
 * - the term (reduce + {1 2 3} initial-value:2) evaluates to 8
 * - the term (reduce + {1 2 3}) evaluates to 6
 *
 * @param c container used for evaluation
 */
protected[function] class ReduceFunction(eval: Evaluator) extends Function with LazyFunction {
    /** Reduce second argument by applying function to accumulated values.
     * @param x input term tuple (f C) or (f C initial-value:a) where f evaluates to a function, C is a collection, and
     *          a is an optional initial accumulator value
     * @return result of applying the function to <code>x</code>
     */
    override def apply(x: Term): Term = x match {
        case Tuple(ft, c: Term, rest@_*) => {
            val f = eval(ft).asFunRef
            val collection = eval(c).asCol
            val initOpt = x.get(Sym("initial-value"))

            val init = initOpt match {
                case Some(t) => t
                case None => collection.head
            }
            val col = initOpt match {
                case Some(t) => collection
                case None => collection.tail
            }

            col.foldLeft(init)((c, x) => f(Tuple(c, x)))
        }
        case _ => throw new IllegalArgumentException(
            s"Bad argument: $x. Need tuple (f c) where f evaluates to " +
              s"a function reference and c is a collection term.")
    }
}

