package org.aiddl.core.scala.container

import org.aiddl.core.scala.representation.{Substitution, Term, Tuple}

case class Entry(t: Term, n: Term, v: Term) {
  def \(s: Substitution): Entry = Entry(t \ s, n \ s, v \ s)

  def asTuple: Tuple = Tuple(t, n, v)

  override def toString(): String = "(" + t + " " + n + " " + v + ")"
}
