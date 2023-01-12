package org.aiddl.core.scala.representation

private[representation] trait VarImpl { self: Var =>
  override def unify(t: Term): Option[Substitution] = if ( this == t ) { Some(new Substitution()) } else { Some(new Substitution(this, t)) }
  override def isGround: Boolean = false

  override def toString: String = {
    val s: Char = this.name(0)
    s match {
      case '_' => "_"
      case _ => "?" + name
    }
  }

  override def asVar: Var = this
}



