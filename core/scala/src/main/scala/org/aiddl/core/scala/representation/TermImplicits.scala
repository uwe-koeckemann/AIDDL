package org.aiddl.core.scala.representation

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.parser.Parser

import scala.language.implicitConversions
/*
given Conversion[Term, KeyVal] with
  def apply(x: Term): KeyVal = x.asKvp

given Conversion[Term, Sym] with
  def apply(x: Term): Sym = x.asSym

given Conversion[Term, Num] with
  def apply(x: Term): Num = x.asNum

given Conversion[Term, CollectionTerm] with
  def apply(x: Term): CollectionTerm = x.asCol

given Conversion[Term, SetTerm] with
  def apply(x: Term): SetTerm = x.asSet

given Conversion[Term, ListTerm] with
  def apply(x: Term): ListTerm = x.asList*/


/*given Conversion[Int, Num] with
  def apply(x: Int): Num = Num(x)

given Conversion[Long, Num] with
  def apply(x: Long): Num = Num(x)

given Conversion[Float, Num] with
  def apply(x: Float): Num = Num(x)

given Conversion[Double, Num] with
  def apply(x: Double): Num = Num(x)*/

/*given Conversion[String, Term] with
  def apply(x: String): Term = {
    val c = new Container()
    val p = new Parser(c)
    p.str(x)
  }*/


object TermImplicits {
  @deprecated
  implicit def int2Num(x: Int): Integer = Integer(x)

  @deprecated
  implicit def float2Num(x: Float): Real = Real(x)

  @deprecated
  implicit def double2Num(x: Double): Real = Real(x)

  @deprecated
  implicit def term2Num(x: Term): Num = x.asNum

  @deprecated
  implicit def term2Symbol(x: Term): Sym = x.asInstanceOf[Sym]

  @deprecated
  implicit def term2KeyVal(x: Term): KeyVal = x.asInstanceOf[KeyVal]
}
