package org.aiddl.core.scala.function

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.CollectionTerm

trait Configurable {
  def config(cfg: CollectionTerm, c: Container): Unit
}
