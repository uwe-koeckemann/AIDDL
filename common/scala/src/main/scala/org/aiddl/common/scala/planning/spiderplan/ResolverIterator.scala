package org.aiddl.common.scala.planning.spiderplan

import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.planning.spiderplan.SpiderPlan.{Consistent, Searching}
import org.aiddl.common.scala.Common.NIL
import org.aiddl.core.scala.representation.TermImplicits.term2KeyVal

trait ResolverIterator {
  def next: Term
}
