package org.aiddl.common.scala.optimization.combinatorial.knapsack

import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm.*
import org.aiddl.common.scala.optimization.combinatorial.CombinatorialOptimizationTerm.*
import org.aiddl.common.scala.optimization.combinatorial.knapsack.Knapsack.*
import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.TermImplicits.term2Num
import org.aiddl.core.scala.representation.TermUnpackImplicits.term2int

class KnapsackToCombinatorialOptimizationConverter extends Function {

  override def apply(x: Term): Term = {
    val cap = x(Capacity).asNum
    val perItemMax = x(PerItemLimit).asNum
    val items = x(Items).asList

    val vars = ListTerm(items.map( i => i(Name) ))
    val scope = vars.asTup
    val domains = ListTerm(items.map( i => {
      perItemMax match {
        case pmi: Integer => KeyVal(i(Name), ListTerm((0 to pmi).map( e => Integer(e)).toVector))
        case InfPos() => KeyVal(i(Name), ListTerm((0 to cap/i(Weight)).map( e => Integer(e)).toVector))
        case _ => throw new IllegalArgumentException(s"Unsupported item maximum $perItemMax. Use Integer or +INF instead.")
      }
    }))

    def cost(x: Term) = {
      (0 until scope.length).map( i => x(i) match {
        case n: Num => n * items(i)(Value)
        case _ => Integer(0)
      }).reduce(_ + _)
    }

    def constraint(x: Term) = {
      Bool((0 until scope.length).map( i => x(i) match {
        case n: Num => n * items(i)(Weight)
        case _ => Integer(0)
      }).reduce(_ + _) <= cap)
    }

    SetTerm(
      KeyVal(Variables, vars),
      KeyVal(Domains, domains),
      KeyVal(Constraints, SetTerm(Tuple(scope, new FunRef(Sym("#knapsack-constraint"), _ => x => constraint(x))))),
      KeyVal(Cost, Tuple(Max, SetTerm(Tuple(scope, new FunRef(Sym("#knapsack-cost"), _ => x => cost(x))))))
    )
  }
}
