package org.aiddl.core.scala.function.`type`

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.{Bool, ListTerm, Term, Tuple}

import org.aiddl.core.scala.representation.BoolImplicits.*
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.TermUnpackImplicits.term2int

class SignatureCheckFunction extends Function {
  def apply(x: Term) = x match {
    case Tuple(Tuple(args@_*), ListTerm(sig))
    => Bool(
      (0 until args.size)
        .forall(
          i => {
            sig(i.min(sig.size - 1)).apply(args(i)).asBool
          }))
    case _ => x
  }
}
