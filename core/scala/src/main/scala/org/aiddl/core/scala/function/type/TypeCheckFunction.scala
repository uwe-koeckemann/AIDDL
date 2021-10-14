package org.aiddl.core.scala.function.`type`

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Function, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.BoolImplicits.*
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.TermUnpackImplicits.term2int
import org.aiddl.core.scala.tools.Logger

class TypeCheckFunction( c: Container ) extends Function {

    val eval = c.getFunctionOrPanic(D.EVAL)

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
            eval(typeTerm\s)
        }
}

