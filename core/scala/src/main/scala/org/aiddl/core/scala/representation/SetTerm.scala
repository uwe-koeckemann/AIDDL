package org.aiddl.core.scala.representation

object SetTerm {
  /**
   * The empty set
   */
  val empty = SetTerm()

  /**
   * Create a new set with some terms in it
   * @param args terms to add to set
   * @return set term containing argument terms
   */
  def apply(args: Term*): SetTerm = SetTerm(args.toSet)
}

/**
 * Represents a set of terms
 * @param set set storing the items contanied in this set
 */
final case class SetTerm(set: Set[Term]) extends CollectionTerm with SetTermImpl