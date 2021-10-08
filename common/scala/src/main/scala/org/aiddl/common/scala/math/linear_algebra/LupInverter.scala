package org.aiddl.common.scala.math.linear_algebra

import scala.collection.mutable.HashMap

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.math.linear_algebra.Matrix
import org.aiddl.common.scala.math.linear_algebra.AiddlMatrix
import org.aiddl.common.scala.math.linear_algebra.LupDecomposition
import org.aiddl.common.scala.math.linear_algebra.LupSolver

import org.aiddl.core.scala.representation.TermImplicits.term2Num
import org.aiddl.core.scala.representation.TermImplicits.double2Num

class LupInverter extends Function {
  def apply( x: Term ): Term = ???

  val lupDecompose = new LupDecomposition
  val lupSolve = new LupSolver

  def apply( a: Matrix ): Matrix = {
    val (l, u, p) = lupDecompose(a)
    val identity = AiddlMatrix.identity(a.m)

    val a_inv = Tuple((0 until a.m).map( i => {
      val sol = lupSolve(l, u, p, identity.row(i))
      Tuple((0 until a.n).map(j => sol(j)): _*)
    }): _*)

    AiddlMatrix(a_inv).t
  }
}