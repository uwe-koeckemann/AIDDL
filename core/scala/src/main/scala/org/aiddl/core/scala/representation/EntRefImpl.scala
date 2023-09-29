package org.aiddl.core.scala.representation

import scala.annotation.targetName

private[representation] trait EntRefImpl { self: EntRef =>

  override def unify(t: Term): Option[Substitution] = t match { 
    case EntRef(tModule, tName, tAlias) if ( mod == tModule && alias == tAlias ) => name unify tName
    case _ => None
  }

  @targetName("substitute")
  override def \(s: Substitution): Term = {
    EntRef((mod\s).asSym, name\s, (alias\s).asSym)
  }

  override def isGround: Boolean = name.isGround

  override def toString(): String = name.toString + "@" + alias.toString 

  override def asEntRef: EntRef = this
}