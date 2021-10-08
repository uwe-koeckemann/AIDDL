package org.aiddl.core.scala.representation

private[representation] trait NanImpl { self: NaN =>

  override def compare(y: Num) = throw new IllegalAccessError("NaN cannot be compared to other numbericals")
    //if ( y.isNan ) 0 else -1

  override def \(s: Substitution): Term = this

  override def +(y: Term): Term = NaN()
  override def -(y: Term): Term = NaN()
  override def *(y: Term): Term = NaN()
  override def /(y: Term): Term = NaN()

  override def isNan = true

  override def equals(obj: Any): Boolean = false
  override def toString = "NaN"
}