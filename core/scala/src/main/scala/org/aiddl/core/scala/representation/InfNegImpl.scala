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
      case NaN() => NaN()
      case _ :Num => InfNeg()
  }

  override def -(y: Num): Num = y match {
      case InfNeg() => NaN()
      case NaN() => NaN()
      case _ : Num => InfNeg()
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

  def toInt: Int = throw new IllegalAccessError(s"$this cannot be converted to Int")
  def toLong: Long = throw new IllegalAccessError(s"$this cannot be converted to Long")
  def toFloat: Float = throw new IllegalAccessError(s"$this cannot be converted to Float")
  def toDouble: Double = throw new IllegalAccessError(s"$this cannot be converted to Double")

  def tryToInt: Option[Int] = None
  def tryToLong: Option[Long] = None
  def tryToFloat: Option[Float] = None
  def tryToDouble: Option[Double] = None
}