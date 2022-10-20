package org.aiddl.core.scala.representation.conversion

import org.aiddl.core.scala.representation.*

given Conversion[Term, KeyVal] = _.asKvp
given Conversion[Term, Sym] = _.asSym
given Conversion[Term, Num] = _.asNum
given Conversion[Term, CollectionTerm] = _.asCol
given Conversion[Term, SetTerm] = _.asSet
given Conversion[Term, ListTerm] = _.asList
given Conversion[Term, Tuple] = _.asTup

given Conversion[Int, Num] = Num(_)
given Conversion[Long, Num] = Num(_)
given Conversion[Float, Num] = Num(_)
given Conversion[Double, Num] = Num(_)