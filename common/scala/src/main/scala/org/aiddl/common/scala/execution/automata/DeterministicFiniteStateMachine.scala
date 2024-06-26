package org.aiddl.common.scala.execution.automata

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.function.Configurable
import org.aiddl.core.scala.function.InterfaceImplementation
import org.aiddl.core.scala.representation.Term
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.Term
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation.CollectionTerm
import org.aiddl.core.scala.representation.SetTerm
import org.aiddl.core.scala.representation.ListTerm
import org.aiddl.core.scala.representation.Bool


class DeterministicFiniteStateMachine extends Function with Initializable with InterfaceImplementation {
    val interfaceUri = Sym("org.aiddl.common.automata.dfa.controller")

    var states = SetTerm()
    var events:SetTerm = SetTerm()
    var transitions: SetTerm = SetTerm()
    var initialState: Term = Sym("NIL")
    var finalStates: SetTerm = SetTerm()
    var s: Term  = Sym("NIL")

    override def init(args: Term): Unit = {
        args match {
            case Tuple(s, e, t, s0, f) => {
                this.states = s.asSet
                this.events = e.asSet
                this.transitions = t.asSet
                this.initialState = s0
                this.finalStates = f.asSet
                this.s = s0
            } 
            case _ => throw new IllegalArgumentException(s"Unsupported input: $args")
        }
    }

    override def apply(x: Term): Term = {
        x match {
            case Tuple(Sym("step"), e) => s = transitions.getOrPanic(Tuple(s, e)); s
            case Tuple(Sym("multi-step"), ListTerm(list)) => s = list.foldLeft(s)( (s, e) => transitions.getOrPanic(Tuple(s, e)) ); s
            case Sym("is-final-state") => Bool(finalStates.contains(s))
            case Sym("current-state") => s
            case Sym("reset") => s = initialState; s
            case _ => throw new IllegalArgumentException(s"Unsupported input: $x")
        }
    }
}