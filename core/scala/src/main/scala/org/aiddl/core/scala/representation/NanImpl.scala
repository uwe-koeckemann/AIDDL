package org.aiddl.core.scala.representation

import scala.annotation.targetName

private[representation] trait NanImpl { self: NaN =>

  override def compare(y: Num) = throw new IllegalAccessError("NaN cannot be compared to other numbericals")
    //if ( y.isNan ) 0 else -1

  override def \(s: Substitution): Term = this

  override def +(y: Num): Num = NaN()
  override def -(y: Num): Num = NaN()
  override def *(y: Num): Num = NaN()
  override def /(y: Num): Num = NaN()
  override def floorDiv(y: Num): Num = NaN()
  override def unary_- = NaN()

  override def isNan = true

  override def equals(obj: Any): Boolean = false
  override def toString = "NaN"
}