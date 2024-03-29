package org.aiddl.common.scala.learning.testing

import scala.util.Random

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.*

class DataShuffler extends Function {
  def apply( data: Term ): Term = ListTerm(Random.shuffle(data.asList))
}
