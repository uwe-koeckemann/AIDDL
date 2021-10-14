package org.aiddl.common.scala.planning.spiderplan

import org.aiddl.core.representation.FunctionReferenceTerm
import org.aiddl.core.scala.representation.Sym
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.representation.*

trait ResolverGenerator extends Function with Initializable  {
  val targets: List[Sym]

  override def apply(x: Term): Term
}
