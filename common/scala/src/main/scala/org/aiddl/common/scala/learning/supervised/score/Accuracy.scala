package org.aiddl.common.scala.learning.supervised.score

import org.aiddl.core.scala.representation._
import org.aiddl.common.scala.learning.supervised.ScoreFunction

class Accuracy extends ScoreFunction {
  def score(yPredicted: ListTerm, yTrue: ListTerm): Num =
    Num(yPredicted.zip(yTrue).count((a, b) => a == b )) / Num(yPredicted.size)
}