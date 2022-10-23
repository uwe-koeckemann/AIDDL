package org.aiddl.common.scala.optimization.combinatorial.knapsack

import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm.*
import org.aiddl.common.scala.optimization.combinatorial.CombinatorialOptimizationTerm.*
import org.aiddl.common.scala.optimization.combinatorial.knapsack.Knapsack.*
import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.*

import scala.util.Random



class KnapsackGenerator extends Function {
  val r = new Random

  override def apply(x: Term): Term = {
    val cap = x(Capacity).asNum
    val perItemMax = x(PerItemLimit).asNum

    val minWeight = x(Weight)(0).intoInt
    val maxWeight = x(Weight)(1).intoInt
    val minValue = x(Value)(0).intoInt
    val maxValue = x(Value)(1).intoInt
    val numItems = x(Items).intoInt

    val items = ListTerm((1 to numItems).map( i => {
      val weight = r.between(minWeight, maxWeight)
      val value = r.between(minValue, maxValue)

      Tuple(
        KeyVal(Name, Sym(s"i$i")),
        KeyVal(Value, Num(value)),
        KeyVal(Weight, Num(weight))
      )
    }).toVector)

    SetTerm(
      KeyVal(Capacity, cap),
      KeyVal(PerItemLimit, perItemMax),
      KeyVal(Items, items)
    )
  }
}
