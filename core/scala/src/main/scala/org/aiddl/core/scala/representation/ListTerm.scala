package org.aiddl.core.scala.representation

import scala.reflect.ClassTag

object ListTerm {
  val empty = ListTerm(Seq.empty)

  @deprecated
  def create(args: Term*): ListTerm = ListTerm(args)

  def apply(args: Term*)(implicit tag: ClassTag[Term]): ListTerm = ListTerm(args)
}

final case class ListTerm(list: Seq[Term])
  extends CollectionTerm
    with Seq[Term]
    //with SeqOps[Term, Seq, ListTerm]
    with ListTermImpl