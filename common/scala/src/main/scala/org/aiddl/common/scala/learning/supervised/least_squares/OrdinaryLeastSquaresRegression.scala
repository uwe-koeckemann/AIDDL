package org.aiddl.common.scala.learning.supervised.least_squares

import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.learning.supervised.Learner
import org.aiddl.common.scala.math.linear_algebra.Matrix
import org.aiddl.common.scala.math.linear_algebra.AiddlMatrix
import org.aiddl.common.scala.math.linear_algebra.LupInverter

class OrdinaryLeastSquaresRegression extends Learner {
    val inv = new LupInverter
    var w: Matrix = _

    def fit(x: ListTerm, y: ListTerm): Term = {
        val X = AiddlMatrix(x)
        val Y = AiddlMatrix(y)
 
        w = inv(X.t * X) * X.t * Y
        w.term
    }

    def predict(x: ListTerm): ListTerm = {
        val y = AiddlMatrix(x)(w)
        ListTerm((0 until y.m).map( i => y(i) ): _*)
    }
}