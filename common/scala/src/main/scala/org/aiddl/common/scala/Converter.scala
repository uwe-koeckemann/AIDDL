package org.aiddl.common.scala

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.Common.NIL
import org.aiddl.core.scala.util.ComboIterator

trait Converter extends Function {
  private var sub: Substitution = new Substitution()

  def encodeAndGetSub( t: Term ): (Term, Substitution)
  def substitution = sub

  final def encode( t: Term ): Term = {
    val (tEnc, sub) = encodeAndGetSub(t)
    this.sub = sub
    tEnc
  }
  final def decode( t: Term ): Term = t \ sub

  def apply( t: Term ): Term = t match {
    case Tuple(Sym("encode"), a) => encode(a)
    case Tuple(Sym("decode"), a) => decode(a)
    case Tuple(Sym("encode-with-sub"), a) => Tuple(encode(a), sub.asTerm)
    case t => encode(t)
  }
}
