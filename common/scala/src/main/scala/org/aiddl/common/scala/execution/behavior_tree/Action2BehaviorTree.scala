package org.aiddl.common.scala.execution.behavior_tree

import org.aiddl.core.scala.representation.Term
import org.aiddl.core.scala.representation.ListTerm
import org.aiddl.core.scala.representation.KeyVal
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.representation.FunRef

class Action2BehaviorTree extends org.aiddl.core.scala.function.Function {
    def apply( a: Term ): Term = {
        val name = a.getOrPanic(Sym("name"))
        val preconditions = a.getOrPanic(Sym("preconditions")).asList
        val effects = a.getOrPanic(Sym("effects")).asList

        val bt_pre = KeyVal(Sym("Sequence"), ListTerm(preconditions.map(e => KeyVal(Sym("Leaf"), e)).appended(KeyVal(Sym("Leaf"), name))))
        val bt = KeyVal(Sym("Select"), ListTerm(effects.map(e => KeyVal(Sym("Leaf"), e)).appended(bt_pre)))

        bt
    } 
}