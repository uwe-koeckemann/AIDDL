package org.aiddl.core.scala.function

import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.representation.{Substitution, Sym, Term}

class NamedFunction(f: Term, eval: Evaluator, args: Option[Term]) extends Function {
  def apply(x: Term): Term = {
    (args match {
      case Some(pattern) => pattern unify x
      case None => {
        val s = new Substitution
        s.add(Sym("#self"), x);
        s.add(Sym("#arg"), x);
        Some(s)
      }
    }) match {
      case Some(s) => eval(f \ s)
      case None => throw new IllegalArgumentException("Could not match argument " + x + " to " + args)
    }
  }
}
