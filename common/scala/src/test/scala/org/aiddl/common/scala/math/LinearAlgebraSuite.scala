package org.aiddl.common.scala.math

import org.scalatest.funsuite.AnyFunSuite

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation._
import org.aiddl.core.scala.container.Entry

import org.aiddl.core.scala.parser.Parser

import org.aiddl.common.scala.math.linear_algebra._

import org.aiddl.core.scala.representation.conversion.given_Conversion_Int_Num
import org.aiddl.core.scala.representation.conversion.given_Conversion_Double_Num

import scala.language.implicitConversions

/**
 *  References:
 *   [1] Thomas H. Cormen, Charles E. Leiserson, Ronald L. Rivest, and Clifford Stein. 2009. Introduction to Algorithms, Third Edition (3rd. ed.). The MIT Press.
 */
class LinearAlgebraSuite extends AnyFunSuite {
    val A = AiddlMatrix(Tuple(
        Tuple(1, 2, 3),
        Tuple(4, 5, 6),
        Tuple(7, 8, 9)))

    val I = AiddlMatrix(Tuple(
        Tuple(1, 0, 0),
        Tuple(0, 1, 0),
        Tuple(0, 0, 1)))

    val x = AiddlMatrix.vec(3, 2, 1)

    def ~= = Matrix.~=(0.000001)_

    test("Size check") {
        assert(A.m == 3)
        assert(A.n == 3)
        assert(x.m == 3)
        assert(x.n == 1)
        assert(x.t.m == 1)
        assert(x.t.n == 3)
    }
    test("Row and column extraction") {
        assert( A.row(0) == AiddlMatrix.row_vec(1, 2, 3))
        assert( A.col(0).t == AiddlMatrix.row_vec(1, 4, 7))
    }
    test("Transpose of transpose equals original") {
        assert( A.t.t == A )
    }
    test("Matrix vector multiplication") {
        assert( A(x) == A * x )
        assert( A(x) == AiddlMatrix.vec(10, 28, 46) )
    }
    test("Matrix scalar multiplication") {
        assert( x * 2 == AiddlMatrix.vec(6, 4, 2) )
    }

    test("LUP decomposition (input and output from [1] page 826)") {
        val A = AiddlMatrix(Tuple(
            Tuple(2, 0, 2, 0.6),
            Tuple(3, 3, 4, -2),
            Tuple(5, 5, 4, 2),
            Tuple(-1, -2, 3.4, -1)
            ))

        val l_true = AiddlMatrix(Tuple(
            Tuple(1, 0, 0, 0),
            Tuple(0.4, 1, 0, 0),
            Tuple(-0.2, 0.5, 1, 0),
            Tuple(0.6, 0, 0.4, 1)
            ))

        val u_true = AiddlMatrix(Tuple(
            Tuple(5, 5, 4, 2),
            Tuple(0, -2, 0.4, -0.2),
            Tuple(0, 0, 4, -0.5),
            Tuple(0, 0, 0, -3)
            ))

        val fLUP = new LupDecomposition()
        val fLupSolve = new LupSolver()
        val (l, u, p) = fLUP(A)

        assert(~=(l, l_true))
        assert(~=(u, u_true))
    }

    test("LUP solver (input and output from [1] page 818)") {
        val fLUP = new LupDecomposition()
        val fLupSolve = new LupSolver()

        val A = AiddlMatrix(Tuple(
            Tuple(1, 2, 0),
            Tuple(3, 4, 4),
            Tuple(5, 6, 3)
            ))

        val l_true = AiddlMatrix(Tuple(
            Tuple(1, 0, 0),
            Tuple(0.2, 1, 0),
            Tuple(0.6, 0.5, 1)
            ))

        val u_true = AiddlMatrix(Tuple(
            Tuple(5, 6, 3),
            Tuple(0, 0.8, -0.6),
            Tuple(0, 0, 2.5)
            ))

        val (l, u, p) = fLUP(A)

        assert(~=(l, l_true))
        assert(~=(u, u_true))

        val b = AiddlMatrix.vec(3, 7, 8)

        val x = fLupSolve(l, u, p, b)
        val x_true = AiddlMatrix.vec(-1.4, 2.2, 0.6)

        assert(~=(x, x_true))
    }

    test("LUP inverter") {
        val fInv = new LupInverter

        val A = AiddlMatrix(Tuple(
            Tuple(4, 7),
            Tuple(2, 6)
        ))

        val A_inv = fInv(A)

        assert(~=(A_inv * A, AiddlMatrix.identity(2)))
    }
}
