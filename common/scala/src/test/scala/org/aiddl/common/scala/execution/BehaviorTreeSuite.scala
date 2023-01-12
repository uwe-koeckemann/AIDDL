package org.aiddl.common.scala.execution

import org.aiddl.common.scala.execution.behavior_tree.{BehaviorTree2Dot, BehaviorTreeRunner}
import org.aiddl.core.scala.container.{Container, Entry}
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.*
import org.scalatest.funsuite.AnyFunSuite

class BehaviorTreeSuite extends AnyFunSuite {
    test("Single successful node execute successful") {
        val n_success = KeyVal(Sym("leaf"), FunRef(Sym("s-node"), _ => Sym("success")))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_success) == Sym("success") )
    }
    test("Single failure node execute fails") {
        val n_failure = KeyVal(Sym("leaf"), FunRef(Sym("f-node"), _ => Sym("failure")))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_failure) == Sym("failure") )
    }
    test("Selector node fails") {
        val n_failure = KeyVal(Sym("leaf"), FunRef(Sym("f-node-"), _ => Sym("failure")))
        val n_selector = KeyVal(Sym("selector"), ListTerm( n_failure, n_failure, n_failure ))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_selector) == Sym("failure") )
    }
    test("Selector node succeeds") {
        val n_failure = KeyVal(Sym("leaf"), FunRef(Sym("f-node"), _ => Sym("failure")))
        val n_success = KeyVal(Sym("leaf"), FunRef(Sym("s-node"), _ => Sym("success")))
        val n_selector = KeyVal(Sym("selector"), ListTerm( n_failure, n_failure, n_success ))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_selector) == Sym("success") )
    }
    test("sequence node fails (first failure)") {
        val n_failure = KeyVal(Sym("leaf"), FunRef(Sym("f-node"), _ => Sym("failure")))
        val n_success = KeyVal(Sym("leaf"), FunRef(Sym("s-node"), _ => Sym("success")))
        val n_sequence = KeyVal(Sym("sequence"), ListTerm( n_failure, n_failure, n_success ))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_sequence) == Sym("failure") )
    }
    test("sequence node fails (second failure)") {
        val n_failure = KeyVal(Sym("leaf"), FunRef(Sym("f-node"), _ => Sym("failure")))
        val n_success = KeyVal(Sym("leaf"), FunRef(Sym("s-node"), _ => Sym("success")))
        val n_sequence = KeyVal(Sym("sequence"), ListTerm( n_success, n_failure, n_failure ))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_sequence) == Sym("failure") )
    }
    test("sequence node fails (third failure)") {
        val n_failure = KeyVal(Sym("leaf"), FunRef(Sym("f-node"), _ => Sym("failure")))
        val n_success = KeyVal(Sym("leaf"), FunRef(Sym("s-node"), _ => Sym("success")))
        val n_sequence = KeyVal(Sym("sequence"), ListTerm( n_success, n_success, n_failure ))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_sequence) == Sym("failure") )
    }
    test("sequence node succeeds") {
        val n_failure = KeyVal(Sym("leaf"), FunRef(Sym("f-node"), _ => Sym("failure")))
        val n_success = KeyVal(Sym("leaf"), FunRef(Sym("s-node"), _ => Sym("success")))
        val n_sequence = KeyVal(Sym("sequence"), ListTerm( n_success, n_success, n_success ))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_sequence) == Sym("success") )
    }

    test("sequence including selector succeeds") {
        val n_failure = KeyVal(Sym("leaf"), FunRef(Sym("f-node"), _ => Sym("failure")))
        val n_success = KeyVal(Sym("leaf"), FunRef(Sym("s-node"), _ => Sym("success")))
        val n_selector = KeyVal(Sym("selector"), ListTerm( n_failure, n_failure, n_success ))
        val n_sequence = KeyVal(Sym("sequence"), ListTerm( n_success, n_selector, n_success ))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_sequence) == Sym("success") )
    }
}