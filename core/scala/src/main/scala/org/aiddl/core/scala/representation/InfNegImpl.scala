package org.aiddl.core.scala.representation

import scala.annotation.targetName

private[representation] trait InfNegImpl { self: InfNeg =>

  @targetName("substitute")
  override def \(s: Substitution): Term = s.get(this)

  override def compare(that: Num): Int = that match {
      case InfNeg() => 0
      case _ => -1
  }

  @targetName("negate")
  override def unary_- = InfPos()

  @targetName("plus")
  override def +(y: Num): Num = y match {
      case InfPos() => NaN()
      case NaN() => NaN()
      case _ :Num => InfNeg()
  }

  @targetName("minus")
  override def -(y: Num): Num = y match {
      case InfNeg() => NaN()
      case NaN() => NaN()
      case _ : Num => InfNeg()
  }

  @targetName("times")
  override def *(y: Num): Num = y match {
    case NaN() => NaN()
    case _: Num => 
      if (y.isZero) { Integer(0) }
      else if (y.isNeg) { InfPos() }
      else { InfNeg() }
  }

  @targetName("dividedBy")
  override def /(y: Num): Num = y match {
      case Integer(y) => if (y < 0) { InfPos() } else { InfNeg() }
      case Rational(n, d) => if (n < 0) { InfPos() } else { InfNeg() }
      case Real(y) => if (y < 0) { InfPos() } else { InfNeg() }
      case _ => NaN()
  }

  override def floorDiv(y: Num): Num = this / y

  override def toString = "-INF"
}