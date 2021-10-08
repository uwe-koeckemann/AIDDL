package org.aiddl.core.scala.representation

private[representation] trait KeyValImpl { self: KeyVal =>

  override def unify(t: Term): Option[Substitution] = t match {
      case KeyVal(oKey, oValue) => (key unify oKey).flatMap( s => s + (value unify oValue))
      case _ => None
  }
  override def isGround: Boolean = key.isGround && value.isGround

  override def \(s: Substitution): Term = KeyVal(key\s, value\s)

  override def toString(): String = key.toString + ":" + value.toString()

  override def asKvp: KeyVal = this
}
