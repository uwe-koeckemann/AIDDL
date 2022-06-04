package org.aiddl.core.scala.function

import org.aiddl.core.scala.eval.Evaluator
import org.aiddl.core.scala.representation.{Substitution, Sym, Term}

/**
 * Functions created via #def are instantiated using this class.
 * Function arguments are applied via substitution. There are two
 * options:
 *
 * 1. If an argument pattern is provided, it will be matched against function arguments to create a substitution.
 * 2. Otherwise, the term <code>#arg</code> will be replaced by the argument of the function.
 *
 * @param f term representing the function
 * @param eval evaluator used to compute the function
 * @param args argument pattern
 */
class NamedFunction(f: Term, eval: Evaluator, args: Option[Term]) extends Function {
  def apply(x: Term): Term = {
    (args match {
      case Some(pattern) => pattern unify x
      case None => {
        val s = new Substitution
        s.add(Sym("#self"), x)
        s.add(Sym("#arg"), x)
        Some(s)
      }
    }) match {
      case Some(s) => eval(f \ s)
      case None => throw new IllegalArgumentException("Could not match argument " + x + " to " + args)
    }
  }
}
