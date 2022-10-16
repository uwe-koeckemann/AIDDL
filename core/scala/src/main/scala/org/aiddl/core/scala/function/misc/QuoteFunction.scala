package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.function.{Function, LazyFunction}
import org.aiddl.core.scala.representation.Term

protected[function] class QuoteFunction extends  Function with LazyFunction {
  def apply(x: Term): Term = x
}
