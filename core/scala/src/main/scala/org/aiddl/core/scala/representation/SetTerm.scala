package org.aiddl.core.scala.representation

object SetTerm {
  val empty = SetTerm()

  def apply(args: Term*): SetTerm = SetTerm(args.toSet)
}

final case class SetTerm(set: Set[Term]) extends CollectionTerm with SetTermImpl