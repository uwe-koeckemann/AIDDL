package org.aiddl.common.scala.execution

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{Sym, Term}

trait ValueProvider extends Function {

  def subscribe(f: Function): Unit
  def query: Term
  def last: Term

  def apply(x: Term): Term = {
    Sym("NIL")
  }
}
