package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.LazyFunction
import org.aiddl.core.scala.representation.Term

class QuoteFunction extends  Function with LazyFunction {
  def apply(x: Term): Term = x
}
