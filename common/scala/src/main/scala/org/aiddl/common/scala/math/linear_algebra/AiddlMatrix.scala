package org.aiddl.common.scala.math.linear_algebra

import scala.collection.immutable.ArraySeq
import scala.collection.mutable

import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.math.linear_algebra.Matrix

import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_Tuple

import scala.language.implicitConversions

object AiddlMatrix {
  def apply( mt: Term ): Matrix = {
    require(mt.forall( row => (row.isInstanceOf[ListTerm] || row.isInstanceOf[Tuple]) && row.size == mt(0).size ))
    new AiddlMatrix(mt, false)
  }
  def identity( n: Int ): Matrix = {
    AiddlMatrix(Tuple(
      {for { i <- 0 until n
             } yield Tuple({for { j <- 0 until n
                                  } yield if (i == j) Num(1.0) else Num(0.0) }: _*)}: _*))
  }

  def vec( col: Num* ): Matrix = col_vec(col: _*)
  def row_vec( row: Num* ): Matrix = new AiddlMatrix(Tuple(Tuple(row: _* )), false)
  def row_vec( row: ListTerm ): Matrix = new AiddlMatrix(Tuple(row), false)
  def row_vec( row: Tuple ): Matrix = new AiddlMatrix(Tuple(row), false)
  def col_vec( col: Num* ): Matrix = new AiddlMatrix(Tuple( col.map( x => Tuple(x) ): _* ), false)
}

class AiddlMatrix(private val mt: Term, private val trans: Boolean) extends Matrix {
  def create( in: Seq[Seq[Term]] ): Matrix = AiddlMatrix(Tuple(in.map( row => Tuple(row: _*)): _*))

  def apply(i: Int , j: Int):Num = if (trans) mt(j)(i).asNum else mt(i)(j).asNum
  def apply(i: Int): Num = if (m == 1) this(0, i) else if (n == 1) this(i, 0) else this(i, i)

  def m = if (trans) mt(0).length else mt.length
  def n = if (trans) mt.length else mt(0).length

  override def row( i: Int ): Matrix = AiddlMatrix.row_vec(mt(i).asList)

  override def t: Matrix = new AiddlMatrix(mt, !trans)

  def term: Term = if (!trans) mt else Tuple(
    {for { i <- 0 until m } yield Tuple({
      for { j <- 0 until n } yield this(i, j)} : _* )} : _*)

  override def equals(other: Any): Boolean = other match {
    case o : Matrix => this.m == o.m && this.n == o.n
      && (0 until m).forall( i => (0 until n).forall( j => this(i, j) == o(i, j))  )
    case _ => false
  }

  override def toString: String = term.mkString("(", "\n ", ")")
}
