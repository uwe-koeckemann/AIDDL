package org.aiddl.core.scala.function.higher_order

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.*

       
class ReduceFunction( c: Container ) extends Function with LazyFunction {
    val eval = c.getFunctionOrPanic(D.EVAL).asInstanceOf[Evaluator]

    override def apply(x: Term): Term = x match {
        case Tuple(ft, c: CollectionTerm, rest@_*) => {
            val f = eval(ft).asFunRef;
            val initOpt = x.get(Sym("initial-value"))

            val init = initOpt match {
                case Some(t) => t
                case None => c.head
            }
            val col = initOpt match {
                case Some(t) => c
                case None => c.tail
            }

            col.foldLeft(init)((c, x) => f(Tuple(c, x)))
        }
        case _ => x
    }
}

