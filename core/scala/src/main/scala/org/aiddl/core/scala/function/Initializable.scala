package org.aiddl.core.scala.function

import org.aiddl.core.scala.representation.Term

trait Initializable {
  def init(args: Term): Unit
}
