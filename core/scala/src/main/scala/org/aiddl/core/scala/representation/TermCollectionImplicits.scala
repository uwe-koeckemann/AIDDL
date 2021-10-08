package org.aiddl.core.scala.representation

import scala.language.implicitConversions

object TermCollectionImplicits {
  implicit def term2CollectionTerm(x: Term): CollectionTerm = x.asInstanceOf[CollectionTerm]

  implicit def term2ListTerm(x: Term): ListTerm = x.asInstanceOf[ListTerm]

  implicit def term2SetTerm(x: Term): SetTerm = x.asInstanceOf[SetTerm]

  implicit def term2Tuple(x: Term): Tuple = x.asInstanceOf[Tuple]


  implicit def seq2ListTerm(x: Seq[Term]): ListTerm = ListTerm(x)

  implicit def seq2Tuple(x: Seq[Term]): Tuple = Tuple(x: _*)

  implicit def set2Term(x: Set[Term]): SetTerm = SetTerm(x)

  implicit def seq2Term(x: Seq[Term]): ListTerm = ListTerm(x)
  //implicit def tup2Term(x: scala.Tuple): Tuple = Tuple(x.toSeq : _*)
}
