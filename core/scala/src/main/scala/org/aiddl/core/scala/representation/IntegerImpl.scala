package org.aiddl.core.scala.representation

private[representation] trait IntegerImpl { self: Integer =>
    override def \(s: Substitution): Term = s.get(this)

    override def asRat: Rational = Rational(x, 1)
    override def asReal: Real = Real(x.doubleValue)
    
    override def compare(that: Num): Int = that match {
        case Integer(y) => (x - y).toInt
        case Rational(n, d) => (x*d - n).toInt
        case Real(y) => x.toDouble compare y
        case InfPos() => -1
        case InfNeg() => 1
    }

    override def unary_- = Integer(-x)

    override def +(y: Term): Num = y match {
        case NaN() => NaN()
        case Integer(y) => Integer(x + y)
        case Rational(n, d) => Rational(n + x*d, d).shorten()
        case Real(y) => Real(x+y)
        case InfPos() => InfPos()
        case InfNeg() => InfNeg()
        case NaN() => NaN()
        case _=> ???
    }

    override def -(y: Term): Num = y match {
        case Integer(y) => Integer(x - y)
        case Rational(n, d) => Rational(x*d-n, d).shorten()
        case Real(y) => Real(x-y)
        case InfPos() => InfNeg()
        case InfNeg() => InfPos()
        case NaN() => NaN()
        case _ => ???
    }

    override def *(y: Term): Num = y match {
        case Integer(y) => Integer(x * y)
        case Rational(n, d) => Rational(n * x, d).shorten()
        case Real(y) => Real(x * y)
        case InfPos() => if (x < 0) { InfNeg() } else if (x == 0) { NaN() } else { InfPos() }
        case InfNeg() => if (x < 0) { InfPos() } else if (x == 0) { NaN() } else { InfNeg() }
        case NaN() => NaN()
        case _ => ???
    }

    override def /(y: Term): Num = y match {
        case y: Num if y.isZero => NaN()
        case Integer(y) => if (x % y == 0) Integer(x / y) else Rational(x, y).shorten()
        case Rational(n, d) => Rational(x*d, n).shorten()
        case Real(y) => Real(x / y)
        case InfPos() => Integer(0)
        case InfNeg() => Integer(0)
        case NaN() => NaN()
        case _ => ???
    }

    override def floorDiv(y: Term): Num = y match {
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
    override def toString(): String = x.toString()
}