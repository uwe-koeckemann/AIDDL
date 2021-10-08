package org.aiddl.common.planning.partial_order

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.Term
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation.ListTerm
import org.aiddl.core.scala.representation.SetTerm

class Sequential2PartialOrderConverter extends Function {

    def apply( args: Term ): Term = args match {
        case Tuple(ListTerm(plan), s0 @ SetTerm(_), g @ SetTerm(_), os @ SetTerm(_)) => {
            

            Sym("Nil")
        }
        case _ => ???        
    }

}

