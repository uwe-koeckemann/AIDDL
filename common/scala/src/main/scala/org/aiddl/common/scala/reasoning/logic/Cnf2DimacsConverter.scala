package org.aiddl.common.scala.reasoning.logic

import scala.collection.mutable.HashMap
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.Converter
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2CollectionTerm
import org.aiddl.core.scala.tools.ComboIterator

import scala.collection.mutable

class Cnf2DimacsConverter extends Converter {
  private val Not1 = Sym("not")
  private val Not2 = Sym("!")

  override def encodeAndGetSub(cnf: Term): (Term, Substitution) = {
    val m = new HashMap[Term, Integer]()
    val s = new Substitution()
    var nextVar = 0

    (
      ListTerm(cnf.map( c => {
        ListTerm(c.map( l => l match {
          case Tuple(op, a) if ( op == Not1 || op == Not2 ) => -m.getOrElseUpdate(a, { nextVar += 1; s.add(nextVar, a); s.add(-nextVar, l); nextVar })
          case _ => m.getOrElseUpdate(l, { nextVar += 1; s.add(nextVar, l); s.add(-nextVar, Tuple(Not1, l)); nextVar })
        }).toSeq)}).toSeq),
      s
    )
  }
}

