package org.aiddl.core.scala.representation

import scala.annotation.targetName

private[representation] trait RealImpl { self: Real =>

    override def asReal: Real = self

    override def \(s: Substitution): Term = s.get(this)

    override def compare(that: Num): Int = that match {
        case Integer(y) => x compare y.toDouble
        case Rational(n, d) => x compare (n.toDouble / d.toDouble)
        case Real(y) => x compare y
        case InfPos() => -1
        case InfNeg() => 1
    }

    override def unary_- = Real(-x)

    override def +(y: Num): Num = y match {
        case Integer(y) => Real(x + y)
        case Rational(n, d) => Real(x + n.toDouble/d.toDouble)
        case Real(y) => Real(x + y)
        case InfPos() => InfPos()
        case InfNeg() => InfNeg()
        case NaN() => NaN()
        case _ => ???
    }

    override def -(y: Num): Num = y match {
        case Integer(y) => Real(x - y)
        case Rational(n, d) => Real(x - n.toDouble/d.toDouble)
        case Real(y) => Real(x - y)
        case InfPos() => InfNeg()
        case InfNeg() => InfPos()
        case NaN() => NaN()
        case _ => ???
    }

    override def *(y: Num): Num = y match {
        case Integer(y) => Real(x * y)
        case Rational(n, d) => Real(x * (n.toDouble/d.toDouble))
        case Real(y) => Real(x * y)  
        case InfPos() => if (x < 0.0) { InfNeg() } else if (x == 0.0) { NaN() } else { InfPos() }
        case InfNeg() => if (x < 0.0) { InfPos() } else if (x == 0.0) { NaN() } else { InfNeg() }
        case NaN() => NaN()
        case _ => ???
    }

    override def /(y: Num): Num = y match {
        case y: Num if y.isZero => NaN()
        case Integer(y) => Real(x / y)
        case Rational(n, d) => Real(x / (n.toDouble/d.toDouble))
        case Real(y) => Real(x / y)  
        case InfPos() => Integer(0)
        case InfNeg() => Integer(0)
        case NaN() => NaN()
        case _ => ???
    }

    override def floorDiv(y: Num): Num = y match {
        case y: Num if y.isZero => NaN()
        case Integer(y) => Real(Math.floor(x / y))
        case Rational(n, d) => Real(Math.floor(x / (n.toDouble/d.toDouble)))
        case Real(y) => Real(Math.floor(x / y))  
        case InfPos() => Integer(0)
        case InfNeg() => Integer(0)
        case NaN() => NaN()
        case _ => ???
    }

    override def equals( other: Any ): Boolean = other match {
        case Integer(y) => y == x
        case Rational(n, d) => n.doubleValue/d == x
        case Real(y) => y == x
        case _ => false
    }

    override def toString(): String = x.toString()
}