package org.aiddl.common.scala.learning.supervised.score

import org.aiddl.common.scala.learning.supervised.ScoreFunction
import org.aiddl.core.scala.representation._

class MeanSquaredError extends ScoreFunction {
    def score(yPredicted: ListTerm, yTrue: ListTerm): Num =
        (0 until yPredicted.size).foldLeft(Num(0))((c, i) => { val e = yPredicted(i).asNum - yTrue(i).asNum; c + e*e } ) / Num(yPredicted.size)
}
