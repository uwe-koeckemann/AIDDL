package org.aiddl.core.scala.representation

import scala.reflect.ClassTag

object ListTerm {
  val empty = ListTerm(Seq.empty)

  /**
   * Create a new list term with some terms in it
   * @param args terms to add to list term
   * @param tag implicit class tag (ignore this)
   * @return list term containing arguments
   */
  def apply(args: Term*)(implicit tag: ClassTag[Term]): ListTerm = ListTerm(args)
}

/**
 * Represents a list of terms
 * @param list internal seq used to store the items
 */
final case class ListTerm(list: Seq[Term])
  extends CollectionTerm
    with Seq[Term]
    with ListTermImpl