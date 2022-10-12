package org.aiddl.core.scala.container

import org.aiddl.core.scala.representation.{Substitution, Term, Tuple}

import scala.annotation.targetName

/**
 * Immutable combination of type, name, and value.
 * @param t type of the value
 * @param n name of the entry
 * @param v value of the entry
 */
case class Entry(t: Term, n: Term, v: Term) {
  /**
   * Substitute type, name and value of this entry and return the substituted entry
   *
   * @param s a substitution
   * @return substituted entry
   */
  @targetName("substitute")
  def \(s: Substitution): Entry = Entry(t \ s, n \ s, v \ s)

  /**
   * Returns this entry as a tuple term
   * @return tuple term representation of this entry
   */
  def asTuple: Tuple = Tuple(t, n, v)

  override def toString: String = "(" + t + " " + n + " " + v + ")"
}
