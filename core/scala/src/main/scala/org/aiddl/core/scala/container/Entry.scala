package org.aiddl.core.scala.container

import org.aiddl.core.scala.representation.{Substitution, Term, Tuple}

import scala.annotation.targetName

/**
 * Immutable combination of type, name, and value.
 * @param typeRef type of the value
 * @param name name of the entry
 * @param value value of the entry
 */
case class Entry(typeRef: Term, name: Term, value: Term) {
  /**
   * Substitute type, name and value of this entry and return the substituted entry
   *
   * @param s a substitution
   * @return substituted entry
   */
  @targetName("substitute")
  def \(s: Substitution): Entry = Entry(typeRef \ s, name \ s, value \ s)

  /**
   * Returns this entry as a tuple term
   * @return tuple term representation of this entry
   */
  def asTuple: Tuple = Tuple(typeRef, name, value)

  override def toString: String = "(" + typeRef + " " + name + " " + value + ")"
}
