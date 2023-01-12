package org.aiddl.common.scala.execution.behavior_tree

import org.aiddl.core.scala.representation.Term
import org.aiddl.core.scala.representation.ListTerm
import org.aiddl.core.scala.representation.KeyVal
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.representation.FunRef

class BehaviorTreeRunner extends org.aiddl.core.scala.function.Function {
    def apply( x: Term ): Term = x match {
        case KeyVal(Sym("sequence"), ListTerm(l))
            => if (l.exists( this(_) == Sym("failure") )) Sym("failure") else Sym("success")
        case KeyVal(Sym("selector"), ListTerm(l))
            => if (l.exists( this(_) == Sym("success") )) Sym("success") else Sym("failure")
        case KeyVal(Sym("leaf"), FunRef(uri, f))
            => f(Sym("Nil"))
        case _ => throw new IllegalArgumentException("Not a legal behavior tree node: " + x)
    } 
}