package org.aiddl.common.scala.reasoning.probabilistic

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.representation.Tuple
import org.aiddl.core.scala.representation._
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.reasoning.probabilistic.ProbabilisticTerm._
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2ListTerm

trait InferenceFunction extends Function with Initializable {

  def apply( x: Term, e: CollectionTerm ): ListTerm

  override def apply(x: Term): Term = this(x(0), x(1))
}
