package org.aiddl.core.scala.representation

import scala.annotation.targetName

private[representation] trait RationalImpl { self: Rational =>
    override def \(s: Substitution): Term = s.get(this)

    override def asReal: Real = Real(n.doubleValue / d.doubleValue)

    def gcd(a: Long, b: Long):Long = if (b == 0) a else gcd(b, a % b)
        
    def shorten(): Rational = { 
        val a = n.abs.max(d.abs); 
        val b = n.abs.min(d.abs); 
        val g = gcd(a, b);
        if (d < 0) Rational(-n/g, -d/g)
        else Rational(n/g, d/g)
    }
        
    override def compare(that: Num): Int = that match {
        case Integer(y) => (n - y*d).toInt
        case Rational(nt, dt) => (n*dt - nt*d).toInt
        case Real(y) => (n.toDouble / d.toDouble) compare y
        case InfPos() => -1
        case InfNeg() => 1
    }

    override def unary_- = Rational(-n, d)

    override def +(y: Num): Num = y match {
        case Integer(y) => Rational(n + y*d, d).shorten()
        case Rational(nt, dt) => Rational(n*dt + nt*d, d*dt).shorten()
        case Real(y) => Real(n.toDouble / d.toDouble + y)
        case InfPos() => InfPos()
        case InfNeg() => InfNeg()
        case NaN() => NaN()
        case _ => ???
    }

    override def -(y: Num): Num = y match {
        case Integer(y) => Rational(n - y*d, d).shorten()
        case Rational(nt, dt) => Rational(n*dt - nt*d, d*dt).shorten()
        case Real(y) => Real(n.toDouble / d.toDouble - y)
        case InfPos() => InfNeg()
        case InfNeg() => InfPos()
        case NaN() => NaN()
        case _ => ???
    }

    override def *(y: Num): Num = y match {
        case Integer(y) => Rational(n*y, d).shorten()
        case Rational(nt, dt) => Rational(n*nt, d*dt).shorten()
        case Real(y) => Real((n.toDouble / d.toDouble) * y)
        case InfPos() => if (n < 0) { InfNeg() } else if (n == 0) { NaN() } else { InfPos() }
        case InfNeg() => if (n < 0) { InfPos() } else if (n == 0) { NaN() } else { InfNeg() }
        case NaN() => NaN()
        case _ => ???
    }

    override def /(y: Num): Num = y match {
        case y: Num if y.isZero => NaN()
        case Integer(y) => Rational(n, d*y).shorten()
        case Rational(nt, dt) => Rational(n*dt, d*nt).shorten()
        case Real(y) => Real((n.toDouble / d.toDouble) / y)
        case InfPos() => Integer(0)
        case InfNeg() => Integer(0)
        case NaN() => NaN()
        case _ => ???
    }
    
    override def floorDiv(y: Num): Num = y match {
        case y: Num if y.isZero => NaN()
        case Integer(y) => Rational(n - n%(d*y), d*y).shorten()
        case Rational(nt, dt) => Rational(n*dt - (n*dt)%(d*nt), d*nt).shorten()
        case Real(y) => Real(Math.floor((n.toDouble / d.toDouble) / y))
        case InfPos() => Integer(0)
        case InfNeg() => Integer(0)
        case NaN() => NaN()
        case _ => ???
    }

    override def equals( other: Any ): Boolean = other match {
        case Integer(y) => Rational(y, 1) == this
        case Rational(n, d) => n == this.n && d == this.d
        case Real(y) => y == n/d.doubleValue
        case _ => false
    }

    override def toString(): String = n.toString() + "/" + d.toString()

    def toInt: Int = (this.n / this.d).toInt
    def toLong: Long = this.n / this.d
    def toFloat: Float = this.n.toFloat / this.d.toFloat
    def toDouble: Double = this.n.toDouble / this.d.toDouble

    def tryToInt: Option[Int] = Some(this.toInt)
    def tryToLong: Option[Long] = Some(this.toLong)
    def tryToFloat: Option[Float] = Some(this.toFloat)
    def tryToDouble: Option[Double] = Some(this.toDouble)
}