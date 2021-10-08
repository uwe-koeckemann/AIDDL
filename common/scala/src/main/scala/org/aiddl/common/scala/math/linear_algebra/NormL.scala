package org.aiddl.common.scala.math.linear_algebra

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.math.linear_algebra.Matrix
import org.aiddl.common.scala.math.linear_algebra.AiddlMatrix
import org.aiddl.common.scala.math.linear_algebra.LupDecomposition
import org.aiddl.common.scala.math.linear_algebra.LupSolver

import org.aiddl.core.scala.representation.TermCollectionImplicits.term2ListTerm
import org.aiddl.core.scala.representation.TermImplicits.term2Num
import org.aiddl.core.scala.representation.TermImplicits.double2Num
import org.aiddl.core.scala.representation.TermUnpackImplicits.term2double

class NormL(n: Int) extends Function {
  def apply( x: Matrix ): Num = Num(
      Math.pow(x.iterRow(0).foldLeft(0.0)((c, t) => c + Math.pow(Math.abs(t), n)), 1.0 / n)
    )

  def apply( x: Term ): Term = Num(Math.pow(x.foldLeft(0.0)( (c, t) => c + Math.pow(Math.abs(t), n) ), 1.0/n))
}
