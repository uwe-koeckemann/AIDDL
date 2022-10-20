package org.aiddl.common.scala.reasoning.logic

import scala.collection.mutable.HashMap
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.Converter
import org.aiddl.core.scala.util.ComboIterator

import scala.collection.mutable

class Cnf2DimacsConverter extends Converter {
  private val Not1 = Sym("not")
  private val Not2 = Sym("!")

  override def encodeAndGetSub(cnf: Term): (Term, Substitution) = {
    val m = new HashMap[Term, Num]()
    val s = new Substitution()
    var nextVar = 0

    (
      ListTerm(cnf.asCol.map( c => {
        ListTerm(c.asCol.map( l => l match {
          case Tuple(op, a) if ( op == Not1 || op == Not2 ) => -m.getOrElseUpdate(a, { nextVar += 1; s.add(Num(nextVar), a); s.add(Num(-nextVar), l); Num(nextVar) })
          case _ => m.getOrElseUpdate(l, { nextVar += 1; s.add(Num(nextVar), l); s.add(Num(-nextVar), Tuple(Not1, l)); Num(nextVar) })
        }).toSeq)}).toSeq),
      s
    )
  }
}

