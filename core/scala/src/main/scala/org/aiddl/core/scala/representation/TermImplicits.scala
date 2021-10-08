package org.aiddl.core.scala.representation

import scala.language.implicitConversions

object TermImplicits {
  implicit def int2Num(x: Int): Integer = Integer(x)

  implicit def float2Num(x: Float): Real = Real(x)

  implicit def double2Num(x: Double): Real = Real(x)

  implicit def term2Num(x: Term): Num = x.asInstanceOf[Num]

  implicit def term2Symbol(x: Term): Sym = x.asInstanceOf[Sym]

  implicit def term2KeyVal(x: Term): KeyVal = x.asInstanceOf[KeyVal]
}
