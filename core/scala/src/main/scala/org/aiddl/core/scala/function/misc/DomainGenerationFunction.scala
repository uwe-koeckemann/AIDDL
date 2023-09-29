package org.aiddl.core.scala.function.misc

import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.function.{Evaluator, Function, LazyFunction, DefaultFunctionUri as D}
import org.aiddl.core.scala.representation.{CollectionTerm, Integer, ListTerm, SetTerm, Sym, Term, Tuple}
import org.aiddl.core.scala.util.ComboIterator

protected[function] class DomainGenerationFunction(eval: Evaluator) extends Function with LazyFunction {
  def apply(x: Term): Term = x match {
    case col: CollectionTerm => SetTerm(evalDomain(col))
    case _ => throw new IllegalArgumentException(s"Bad argument: $x. Expected collection term.")
  }

  private def evalDomain(col: CollectionTerm): Set[Term] =
    if (col.containsKey(Sym("min"))) {
      val min = eval(col(Sym("min")))
      val inc = eval(col.getOrElse(Sym("inc"), Integer(1))).asNum
      val max = eval(col(Sym("max"))).asNum
      var xs: List[Term] = Nil
      var c = min.asNum

      while (c <= max) {
        xs = c :: xs
        c += inc
      }
      xs.toSet

    } else {
      col.map(e => eval.apply(e)).flatMap(
        e => e match {
          case col: CollectionTerm => evalDomain(col)
          case tup: Tuple => {
            val choices = tup.map(x => eval(x) match {
              case ListTerm(list) => list
              case SetTerm(set) => set.toSeq
              case _ => List(x)
            }).toSeq
            val comboIt = new ComboIterator(choices);
            for (combo <- comboIt) yield Tuple(combo: _*)
          }
          case _ => List(e).toSet
        }
      ).toSet
    }
}
