package org.aiddl.common.scala.planning.spiderplan

import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.planning.spiderplan.SpiderPlan.{Consistent, Searching}
import org.aiddl.common.scala.Common.NIL
import org.aiddl.core.scala.representation.TermImplicits.term2KeyVal

class ResolverList( consistent: Boolean, var rs: List[Term] ) extends Function with Iterator[Term] {
  def hasNext = rs.isEmpty
  def next = rs match {
    case r :: tail => {rs = tail; r}
    case Nil => throw new NoSuchElementException()
  }

  def apply(x: Term): Term = {
    if (consistent && hasNext)
      Tuple(Consistent, next)
    else if (hasNext)
      Tuple(Searching, next)
    else
      NIL
  }
}
