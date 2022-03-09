package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.core.scala.representation._
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.search.TreeSearch

import org.aiddl.core.scala.representation.BoolImplicits.term2Boolean
import org.aiddl.core.scala.representation.TermImplicits._
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2CollectionTerm

import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm._

class CspSolver extends TreeSearch {
  private var vars: CollectionTerm = _
  private var doms: CollectionTerm = _
  private var cons: CollectionTerm = _

  override def init( csp: Term ) = {
    vars = csp(Variables)
    doms = csp(Domains)
    cons = csp(Constraints)

    super.init(csp)
  }

  override def expand: Option[Seq[Term]] =
    vars.find( x => !choice.exists( a => a.key == x ) )
        .flatMap( x => Some(ListTerm(doms(x).map( v => KeyVal(x, v)  ).toSeq)))

  override def isConsistent: Boolean = {
    val sub = new Substitution()
    choice.foreach( a => sub.add(a.key, a.value) )
    cons.forall( c => {
      val args = c(0)\sub
      val pCon = c(1)
      if ( args.isGround ) pCon(args) else true
    })
  }
}
