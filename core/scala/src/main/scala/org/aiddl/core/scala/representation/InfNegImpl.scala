package org.aiddl.core.scala.representation

import org.aiddl.core.scala.representation.TermImplicits.*

import scala.annotation.targetName

private[representation] trait InfNegImpl { self: InfNeg =>

  override def \(s: Substitution): Term = s.get(this)

  override def compare(that: Num): Int = that match {
      case InfNeg() => 0
      case _ => -1
  }

  override def unary_- = InfPos()

  override def +(y: Num): Num = y match {
      case InfPos() => NaN()
      case _ :Num => InfNeg()
      case _ => NaN()
  }

  override def -(y: Num): Num = y match {
      case InfNeg() => NaN()
      case _ : Num => InfNeg()
      case _ => NaN()
  }

  override def *(y: Num): Num = y match {
    case NaN() => NaN()
    case _: Num => 
      if (y.isZero) { Integer(0) }
      else if (y.isNeg) { InfPos() }
      else { InfNeg() }
  }

  override def /(y: Num): Num = y match {
      case Integer(y) => if (y < 0) { InfPos() } else { InfNeg() }
      case Rational(n, d) => if (n < 0) { InfPos() } else { InfNeg() }
      case Real(y) => if (y < 0) { InfPos() } else { InfNeg() }
      case _ => NaN()
  }

  override def floorDiv(y: Num): Num = this / y

  override def toString = "-INF"
}