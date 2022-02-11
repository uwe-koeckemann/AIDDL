package org.aiddl.common.scala.execution

import org.scalatest.funsuite.AnyFunSuite
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation.Var
import org.aiddl.core.scala.parser.Parser
import org.aiddl.core.scala.representation.ListTerm
import org.aiddl.core.scala.representation.Bool
import org.aiddl.core.scala.representation.Term

import org.aiddl.common.scala.execution.behavior_tree.BehaviorTreeRunner
import org.aiddl.core.scala.representation.KeyVal
import org.aiddl.core.scala.representation.FunRef
import org.aiddl.common.scala.execution.behavior_tree.BehaviorTree2Dot

class BehaviorTreeSuite extends AnyFunSuite {
    test("Single successful node execute successful") {
        val n_success = KeyVal(Sym("Leaf"), FunRef(Sym("s-node"), _ => Sym("success")))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_success) == Sym("success") )
    }
    test("Single failure node execute fails") {
        val n_failure = KeyVal(Sym("Leaf"), FunRef(Sym("f-node"), _ => Sym("failure")))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_failure) == Sym("failure") )
    }
    test("Selector node fails") {
        val n_failure = KeyVal(Sym("Leaf"), FunRef(Sym("f-node-"), _ => Sym("failure")))
        val n_selector = KeyVal(Sym("Select"), ListTerm( n_failure, n_failure, n_failure ))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_selector) == Sym("failure") )
    }
    test("Selector node succeeds") {
        val n_failure = KeyVal(Sym("Leaf"), FunRef(Sym("f-node"), _ => Sym("failure")))
        val n_success = KeyVal(Sym("Leaf"), FunRef(Sym("s-node"), _ => Sym("success")))
        val n_selector = KeyVal(Sym("Select"), ListTerm( n_failure, n_failure, n_success ))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_selector) == Sym("success") )
    }
    test("Sequence node fails (first failure)") {
        val n_failure = KeyVal(Sym("Leaf"), FunRef(Sym("f-node"), _ => Sym("failure")))
        val n_success = KeyVal(Sym("Leaf"), FunRef(Sym("s-node"), _ => Sym("success")))
        val n_sequence = KeyVal(Sym("Sequence"), ListTerm( n_failure, n_failure, n_success ))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_sequence) == Sym("failure") )
    }
    test("Sequence node fails (second failure)") {
        val n_failure = KeyVal(Sym("Leaf"), FunRef(Sym("f-node"), _ => Sym("failure")))
        val n_success = KeyVal(Sym("Leaf"), FunRef(Sym("s-node"), _ => Sym("success")))
        val n_sequence = KeyVal(Sym("Sequence"), ListTerm( n_success, n_failure, n_failure ))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_sequence) == Sym("failure") )
    }
    test("Sequence node fails (third failure)") {
        val n_failure = KeyVal(Sym("Leaf"), FunRef(Sym("f-node"), _ => Sym("failure")))
        val n_success = KeyVal(Sym("Leaf"), FunRef(Sym("s-node"), _ => Sym("success")))
        val n_sequence = KeyVal(Sym("Sequence"), ListTerm( n_success, n_success, n_failure ))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_sequence) == Sym("failure") )
    }
    test("Sequence node succeeds") {
        val n_failure = KeyVal(Sym("Leaf"), FunRef(Sym("f-node"), _ => Sym("failure")))
        val n_success = KeyVal(Sym("Leaf"), FunRef(Sym("s-node"), _ => Sym("success")))
        val n_sequence = KeyVal(Sym("Sequence"), ListTerm( n_success, n_success, n_success ))
        val f_BT = new BehaviorTreeRunner
        assert( f_BT(n_sequence) == Sym("success") )
    }

    test("Sequence including selector succeeds") {
        val n_failure = KeyVal(Sym("Leaf"), FunRef(Sym("f-node"), _ => Sym("failure")))
        val n_success = KeyVal(Sym("Leaf"), FunRef(Sym("s-node"), _ => Sym("success")))
        val n_selector = KeyVal(Sym("Select"), ListTerm( n_failure, n_failure, n_success ))        
        val n_sequence = KeyVal(Sym("Sequence"), ListTerm( n_success, n_selector, n_success ))        
        val f_BT = new BehaviorTreeRunner

        val f_BT2DOT = new BehaviorTree2Dot
        val s = f_BT2DOT(n_sequence)
        //println("DOT:" + s)
        //f_BT2DOT.toFile(n_sequence, "bt.dot")

        assert( f_BT(n_sequence) == Sym("success") )
    }
}