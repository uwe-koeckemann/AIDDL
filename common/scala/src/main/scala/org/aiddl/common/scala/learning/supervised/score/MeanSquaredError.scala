package org.aiddl.common.scala.learning.supervised.score

import org.aiddl.common.scala.learning.supervised.ScoreFunction
import org.aiddl.core.scala.representation._

class MeanSquaredError extends ScoreFunction {
    def score(y_p: ListTerm, y_t: ListTerm): Num = 
        (0 until y_p.size).foldLeft(Num(0))( (c, i) => { val e = y_p(i).asNum - y_t(i).asNum; c + e*e } ) / Num(y_p.size)
}
