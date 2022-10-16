package org.aiddl.core.scala.representation

import org.aiddl.core.scala.representation.TermImplicits.*

import scala.annotation.targetName

private[representation] trait EntRefImpl { self: EntRef =>

  override def unify(t: Term): Option[Substitution] = t match { 
    case EntRef(tmod, tname, talias) if ( mod == tmod && alias == talias ) => name unify tname
    case _ => None
  }

  @targetName("substitute")
  override def \(s: Substitution): Term = {
    EntRef(mod\s, name\s, alias\s)
  }

  override def isGround: Boolean = name.isGround

  override def toString(): String = name.toString + "@" + alias.toString 

  override def asEntRef: EntRef = this
}