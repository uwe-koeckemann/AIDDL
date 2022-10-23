package org.aiddl.core.scala.function.`type`

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function.{Function, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.Logger
import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_Sym
import scala.language.implicitConversions

protected[function] class TypeCheckFunction(c: Container) extends Function {
    def apply( x: Term ): Term = {
      val r = x match {
        case Tuple(value, typeTerm) if typeTerm.isInstanceOf[CollectionTerm] => {
          Bool(typeTerm.asCol.exists(t => checkInternal(t, value) == Bool(true)))
        }
        case Tuple(value, typeTerm) => checkInternal(typeTerm, value)
        case _ => throw new IllegalArgumentException(s"Type check failed for $x")
      }
      r
    }

    private def checkInternal( typeTerm: Term, target: Term ): Term = 
        if ( typeTerm.isInstanceOf[FunRef] ) 
          typeTerm(target)
        else if ( typeTerm.isInstanceOf[Sym] ) {
          val f = FunRef(typeTerm, c.getFunctionOrPanic(typeTerm))
          f(target)
        } else {
            val s = new Substitution()
            s.add(Sym("#self"), target)
            s.add(Sym("#arg"), target)
            c.eval(typeTerm\s)
        }
}


