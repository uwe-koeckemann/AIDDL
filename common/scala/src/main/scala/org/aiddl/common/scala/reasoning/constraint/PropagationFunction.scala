package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.representation.{CollectionTerm, Term}

trait PropagationFunction extends Initializable {

  override def init(csp: Term): Unit

  def propagate(assignment: List[Term], domains: CollectionTerm): Option[CollectionTerm]
}
