package org.aiddl.common.scala.math.linear_algebra

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.math.linear_algebra.Matrix
import org.aiddl.common.scala.math.linear_algebra.AiddlMatrix
import org.aiddl.common.scala.math.linear_algebra.LupDecomposition
import org.aiddl.common.scala.math.linear_algebra.LupSolver

class NormL(n: Int) extends Function {
  def apply( x: Matrix ): Num = Num(
      Math.pow(x.iterRow(0).foldLeft(0.0)((c, t) => c + Math.pow(Math.abs(t.asNum.toDouble), n)), 1.0 / n)
    )

  def apply( x: Term ): Term = Num(Math.pow(x.asCol.foldLeft(0.0)( (c, t) => c + Math.pow(Math.abs(t.asNum.toDouble), n) ), 1.0/n))
}
