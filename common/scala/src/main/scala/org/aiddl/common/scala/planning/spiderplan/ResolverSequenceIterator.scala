package org.aiddl.common.scala.planning.spiderplan

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.planning.spiderplan.SpiderPlan.{Consistent, Searching}
import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.TermImplicits.term2KeyVal

class ResolverSequenceIterator(consistent: Boolean, var rs: Seq[Term] ) extends ResolverIterator {
  private def hasNextResolver = !rs.isEmpty
  private def nextResolver = rs match {
    case r :: tail => {rs = tail; r}
    case Nil => throw new NoSuchElementException()
  }

  def next: Term = {
    println(rs)
    val r = if (consistent && hasNextResolver)
      Tuple(Consistent, nextResolver)
    else if (hasNextResolver)
      Tuple(Searching, nextResolver)
    else
      NIL
    println(r)
    r
  }
}
