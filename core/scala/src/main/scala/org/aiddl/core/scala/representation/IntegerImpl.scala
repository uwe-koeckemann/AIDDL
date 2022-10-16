package org.aiddl.core.scala.representation

import scala.annotation.targetName

private[representation] trait IntegerImpl { self: Integer =>
    @targetName("substitute")
    override def \(s: Substitution): Term = s.get(this)


    override def compare(that: Num): Int = that match {
        case Integer(y) => (x - y).toInt
        case Rational(n, d) => (x*d - n).toInt
        case Real(y) => x.toDouble compare y
        case InfPos() => -1
        case InfNeg() => 1
    }

    @targetName("negate")
    override def unary_- = Integer(-x)

    @targetName("plus")
    override def +(y: Num): Num = y match {
        case Integer(y) => Integer(x + y)
        case Rational(n, d) => Rational(n + x*d, d).shorten()
        case Real(y) => Real(x+y)
        case InfPos() => InfPos()
        case InfNeg() => InfNeg()
        case NaN() => NaN()
        case _=> ???
    }

    @targetName("minus")
    override def -(y: Num): Num = y match {
        case Integer(y) => Integer(x - y)
        case Rational(n, d) => Rational(x*d-n, d).shorten()
        case Real(y) => Real(x-y)
        case InfPos() => InfNeg()
        case InfNeg() => InfPos()
        case NaN() => NaN()
        case _ => ???
    }

    @targetName("times")
    override def *(y: Num): Num = y match {
        case Integer(y) => Integer(x * y)
        case Rational(n, d) => Rational(n * x, d).shorten()
        case Real(y) => Real(x * y)
        case InfPos() => if (x < 0) { InfNeg() } else if (x == 0) { NaN() } else { InfPos() }
        case InfNeg() => if (x < 0) { InfPos() } else if (x == 0) { NaN() } else { InfNeg() }
        case NaN() => NaN()
        case _ => ???
    }

    @targetName("dividedBy")
    override def /(y: Num): Num = y match {
        case y: Num if y.isZero => NaN()
        case Integer(y) => if (x % y == 0) Integer(x / y) else Rational(x, y).shorten()
        case Rational(n, d) => Rational(x*d, n).shorten()
        case Real(y) => Real(x / y)
        case InfPos() => Integer(0)
        case InfNeg() => Integer(0)
        case NaN() => NaN()
        case _ => ???
    }

    override def floorDiv(y: Num): Num = y match {
        case y: Num if y.isZero => NaN()
        case Integer(y) => Integer(x / y)
        case Rational(n, d) => Rational(x*d - (x*d)%n , n).shorten()
        case Real(y) => Real(Math.floor(x / y))
        case InfPos() => Integer(0)
        case InfNeg() => Integer(0)
        case NaN() => NaN()
        case _ => ???
    }

    override def equals( other: Any ): Boolean = other match {
        case Integer(y) => x == y
        case Rational(n, d) => Rational(x, 1) == other
        case Real(y) => x == y
        case _ => false
    }

    override def asInt: Integer = this
    override def asRat: Rational = Rational(x, 1)
    override def asReal: Real = Real(x.doubleValue)

    override def toString(): String = x.toString()

    override def toInt: Int = this.x.toInt
    override def toLong: Long = this.x
    override def toFloat: Float = this.x.toFloat
    override def toDouble: Double = this.x.toDouble

    override def tryToInt: Option[Int] = Some(this.toInt)
    override def tryToLong: Option[Long] = Some(this.toLong)
    override def tryToFloat: Option[Float] = Some(this.toFloat)
    override def tryToDouble: Option[Double] = Some(this.toDouble)
}