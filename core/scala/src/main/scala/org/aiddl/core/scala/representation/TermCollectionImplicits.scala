package org.aiddl.core.scala.representation

import scala.language.implicitConversions

object TermCollectionImplicits {
  @deprecated
  implicit def term2CollectionTerm(x: Term): CollectionTerm = x.asInstanceOf[CollectionTerm]
  @deprecated
  implicit def term2ListTerm(x: Term): ListTerm = x.asList
  @deprecated
  implicit def term2SetTerm(x: Term): SetTerm = x.asSet
  @deprecated
  implicit def term2Tuple(x: Term): Tuple = x.asTup
  @deprecated
  implicit def seq2ListTerm(x: Seq[Term]): ListTerm = ListTerm(x)
  @deprecated
  implicit def seq2Tuple(x: Seq[Term]): Tuple = Tuple(x: _*)
  @deprecated
  implicit def set2Term(x: Set[Term]): SetTerm = SetTerm(x)
  @deprecated
  implicit def seq2Term(x: Seq[Term]): ListTerm = ListTerm(x)
}
