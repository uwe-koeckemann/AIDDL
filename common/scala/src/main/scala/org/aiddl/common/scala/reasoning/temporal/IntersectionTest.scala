package org.aiddl.common.scala.reasoning.temporal

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation._

import org.aiddl.core.scala.representation.TermCollectionImplicits.term2CollectionTerm

import org.aiddl.core.scala.representation.given_Conversion_Term_Num

class IntersectionTest extends Function {
  def apply( is: CollectionTerm, d: CollectionTerm ): Boolean = {
    val overlap = is.foldLeft( (InfNeg(): Term, InfPos(): Term) )( (c, i) => {
      ( if ( d(i)(0)(0) > c(0) ) d(i)(0)(0) else c(0),
        if ( d(i)(1)(0) < c(1) ) d(i)(1)(0) else c(0) ) })
    overlap(1) < overlap(0)
  }

  def apply( x: Term ): Term = Bool(this(x(0), x(1)))
}
