package org.aiddl.common.scala.math.linear_algebra

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation._
import scala.collection.mutable.HashMap

import org.aiddl.common.scala.math.linear_algebra.Matrix
import org.aiddl.common.scala.math.linear_algebra.AiddlMatrix
import org.aiddl.common.scala.math.linear_algebra.LupDecomposition

import org.aiddl.core.scala.representation.TermImplicits.term2Num
import org.aiddl.core.scala.representation.TermImplicits.double2Num


class LupSolver extends Function {
    def apply( x: Term ): Term = ???

    def apply( l: Matrix, u: Matrix, pi: Matrix, b: Matrix ): Matrix = {
        val x = Array.fill(l.m)(Num(0))
        val y = Array.fill(l.m)(Num(0))

        (0 until l.m).foreach( i => {
            val s = (0 until i).foldLeft(Num(0))( (c, j) => c + l(i, j) * y(j) )
            val y_i = b(pi(i).asInt.x.intValue) - s
            y.update(i, y_i)
        })

        (l.m-1 to 0 by -1).foreach( i => {
            val s = (i+1 until l.m).foldLeft(Num(0))( (c, j) => c + u(i, j) * x(j) )
            val x_i = (y(i) - s) / u(i, i)
            x.update(i, x_i)
        })
        AiddlMatrix.vec(x.toSeq: _*)
    }
}   


