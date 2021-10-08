package org.aiddl.core.scala.representation

import org.aiddl.core.scala.representation.TermImplicits._

private[representation] trait EntRefImpl { self: EntRef =>

  override def unify(t: Term): Option[Substitution] = t match { 
    case EntRef(tmod, tname, talias) if ( mod == tmod && alias == talias ) => name unify tname
    case _ => None
  }

  override def \(s: Substitution): Term = {
    EntRef(mod\s, name\s, alias\s)
  }

  override def isGround: Boolean = name.isGround

  override def toString(): String = name.toString + "@" + alias.toString 

  override def asEntRef: EntRef = this
}