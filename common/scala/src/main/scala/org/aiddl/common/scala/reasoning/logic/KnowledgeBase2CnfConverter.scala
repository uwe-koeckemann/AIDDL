package org.aiddl.common.scala.reasoning.logic

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.Common.NIL
import org.aiddl.core.scala.util.ComboIterator

import Term.given_Conversion_Term_Sym

import scala.language.implicitConversions

class KnowledgeBase2CnfConverter extends Function {

  private val And1 = Sym("and")
  private val And2 = Sym("&")
  private val Or1 = Sym("or")
  private val Or2 = Sym("|")
  private val Xor = Sym("xor")
  private val Implies1 = Sym("implies")
  private val Implies2 = Sym("=>")
  private val If1 = Sym("if")
  private val If2 = Sym("<=")
  private val Equivalent1 = Sym("equivalent")
  private val Equivalent2 = Sym("<=>")
  private val Not1 = Sym("not")
  private val Not2 = Sym("!")
  private val operators = Set(And1, And2, Or1, Or2, Xor, Implies1, Implies2, If1, If2,
    Equivalent1, Equivalent2, Not1, Not2)

  override def apply(x: Term): Term = {
    val andForm = Tuple(And2 :: x.asCol.map( x => x ).toList: _*)
    val cnf = convert(andForm)
    ListTerm(cnf.map( t => t match { case s : ListTerm => ListTerm(s.distinct) case _ => ListTerm(t) }).distinct)
  }

  private def convert( f: Term ): List[Term] = {
    f match {
      case Tuple(op, x: _*) if (op == And1 || op == And2) => x.flatMap(convert(_)).toList
      case Tuple(op, x: _*) if (op == Or1 || op == Or2) => {
        val choices = x.map(convert(_).toSeq)
        val cIter = new ComboIterator(choices)
        cIter.map( combo => {
          val orForm: List[Term] = combo.flatMap( t => t match {
            case Tuple(op, args: _*) if (op == Or1 || op == Or2) => args.toSet
            case x : ListTerm => x.list
            case o => List(o)
          }).toList
          ListTerm(orForm)
        }).toList}
      case Tuple(Xor, x: _*) => {
        convert(Tuple(Or1 :: x.map( c_i => {
          Tuple(And1 :: x.map( c_j => {
            if ( c_i == c_j ) c_j else Tuple(Not1, c_j)
          }).toList : _*)
        }).toList: _*)) }
      case Tuple(op, a, b) if (op == Implies1 || op == Implies2) => convert(Tuple(Or1, Tuple(Not1, a), b))
      case Tuple(op, a, b) if (op == If1 || op == If2) => convert(Tuple(Or1, a, Tuple(Not1, b)))
      case Tuple(op, a, b) if (op == Equivalent1 || op == Equivalent2) => convert(Tuple(Or1, Tuple(And1, a, b), Tuple(And1, Tuple(Not1, a), Tuple(Not1, b))))
      case Tuple(op, a) if (op == Not1 || op == Not2) => {
        a match {
          case Tuple(opInt, x: _*) if (opInt == Or1 || opInt == Or2) =>
            convert(Tuple(And1 :: x.map( t => Tuple(Not1, t)).toList: _*))
          case Tuple(opInt, x: _*) if (opInt == And1 || opInt == And2) =>
            convert(Tuple(Or1 :: x.map( t => Tuple(Not1, t)).toList: _*))
          case Tuple(opInt, b) if (opInt == Not1 || opInt == Not2) => convert(b)
          case Tuple(opInt, x: _*) if (operators.contains(opInt)) =>
            convert(Tuple(Or1 :: convert(a).map(t => Tuple(Not1, t)).toList: _*))
          case _ => List(f)
        }
      }
      case f => List(f)
    }
  }
}
