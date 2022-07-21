package org.aiddl.core.scala.representation

import org.aiddl.core.scala.representation.TermImplicits.*

import scala.annotation.targetName

private[representation] trait InfPosImpl {
  self: InfPos =>

  override def \(s: Substitution): Term = s.get(this)

  override def compare(that: Num): Int = that match {
    case InfPos() => 0
    case _ => 1
  }

  override def unary_- = InfNeg()

  override def +(y: Num): Num = y match {
    case InfNeg() => NaN()
    case NaN() => NaN()
    case _: Num => InfPos()
  }

  override def -(y: Num): Num = y match {
    case InfPos() => NaN()
    case NaN() => NaN()
    case _: Num => InfPos()
  }

  override def *(y: Num): Num = y match {
    case NaN() => NaN()
    case _: Num =>
      if (y.asNum.isZero) {
        Integer(0)
      }
      else if (y.asNum.isNeg) {
        InfNeg()
      }
      else {
        InfPos()
      }
  }

  override def /(y: Num): Num = y match {
    case Integer(y) => if (y < 0) {
      InfNeg()
    } else {
      InfPos()
    }
    case Rational(n, d) => if (n < 0) {
      InfNeg()
    } else {
      InfPos()
    }
    case Real(y) => if (y < 0) {
      InfNeg()
    } else {
      InfPos()
    }
    case _ => NaN()
  }

  override def floorDiv(y: Num): Num = this / y

  override def toString = "+INF"
}
