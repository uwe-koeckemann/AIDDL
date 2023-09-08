package org.aiddl.core.scala.test

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.container.Entry
import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.function
import org.aiddl.core.scala.function.{Configurable, DefaultFunctionUri, Function, Initializable}
import org.aiddl.core.scala.representation.{CollectionTerm, KeyVal, ListTerm, Num, SetTerm, Sym, Term, Tuple, Var}
import org.aiddl.core.scala.parser.Parser
class TestFunction() extends Function with Configurable with Initializable {
  var cfgTerm: CollectionTerm = _
  var initTerm: Term = _

  override def config(cfg: CollectionTerm, c: Container): Unit =
    this.cfgTerm = cfg

  override def init(args: Term): Unit =
    this.initTerm = args

  override def apply(x: Term): Term =
    Tuple(cfgTerm, initTerm)
}
