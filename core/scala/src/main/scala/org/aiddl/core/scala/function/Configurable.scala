package org.aiddl.core.scala.function

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.CollectionTerm

/**
 * Function can be configured with a given term in the context of a container.
 */
trait Configurable {
  /** Apply configuration to this object
   * @param cfg configuration term. usually a collection of key-value pairs.
   * @param c container
   */
  def config(cfg: CollectionTerm, c: Container): Unit
}
