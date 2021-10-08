package org.aiddl.core.scala.representation

import org.aiddl.core.scala.representation.TermImplicits.*

private[representation] trait InfPosImpl {
  self: InfPos =>

  override def \(s: Substitution): Term = s.get(this)

  override def compare(that: Num): Int = that match {
    case InfPos() => 0
    case _ => 1
  }

  override def unary_- = InfNeg()

  override def +(y: Term): Num = y match {
    case InfNeg() => NaN()
    case _: Num => InfPos()
    case _ => NaN()
  }

  override def -(y: Term): Num = y match {
    case InfPos() => NaN()
    case _: Num => InfPos()
    case _ => NaN()
  }

  override def *(y: Term): Num = y match {
    case _: Num =>
      if (y > Real(-1.0) && y < Real(1.0)) {
        Integer(0)
      }
      else if (y < Real(0.0)) {
        InfNeg()
      }
      else {
        InfPos()
      }
    case _ => NaN()
  }

  override def /(y: Term): Num = y match {
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
    case _ => ???
  }

  override def toString = "+INF"
}
