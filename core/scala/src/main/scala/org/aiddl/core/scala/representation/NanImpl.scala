package org.aiddl.core.scala.representation

import scala.annotation.targetName

private[representation] trait NanImpl { self: NaN =>
  override def compare(y: Num) = throw new IllegalAccessError("NaN cannot be compared to other numerical terms")

  @targetName("substitute")
  override def \(s: Substitution): Term = this

  @targetName("plus")
  override def +(y: Num): Num = NaN()
  @targetName("minus")
  override def -(y: Num): Num = NaN()
  @targetName("times")
  override def *(y: Num): Num = NaN()
  @targetName("dividedBy")
  override def /(y: Num): Num = NaN()
  override def floorDiv(y: Num): Num = NaN()
  @targetName("negate")
  override def unary_- = NaN()

  override def isNan = true

  override def equals(obj: Any): Boolean = false
  override def toString = "NaN"
}