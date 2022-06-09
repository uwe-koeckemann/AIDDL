package org.aiddl.core.scala.function

import org.aiddl.core.scala.representation.Term

/**
 * Trait for functions that can be initialized (e.g., search or data structures).
 */
trait Initializable {
  /**
   * Initialize the function with a given term.
   * @param args initialization arguments
   */
  def init(args: Term): Unit
}
