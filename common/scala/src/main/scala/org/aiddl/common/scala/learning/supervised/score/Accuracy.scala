package org.aiddl.common.scala.learning.supervised.score

import org.aiddl.core.scala.representation._
import org.aiddl.common.scala.learning.supervised.ScoreFunction

class Accuracy extends ScoreFunction {
  def score(y_p: ListTerm, y_t: ListTerm): Num =
    Num(y_p.zip(y_t).count( (a, b) => a == b )) / Num(y_p.size)
}