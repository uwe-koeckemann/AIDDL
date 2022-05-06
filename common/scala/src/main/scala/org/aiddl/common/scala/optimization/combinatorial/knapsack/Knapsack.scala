package org.aiddl.common.scala.optimization.combinatorial.knapsack

import org.aiddl.core.scala.representation.TermUnpackImplicits.term2int
import org.aiddl.core.scala.representation.{Num, Sym, Term}

object Knapsack {
  val Items = Sym("items")
  val Name = Sym("name")
  val Weight = Sym("weight")
  val Value = Sym("value")

  val Capacity = Sym("capacity")
  val PerItemLimit = Sym("per-item-limit")

  def remainingCostEstGeneratorFillWithBest(problem: Term): List[Term] => Num = {
    val bestValForWeight = problem(Items).asCol.map(item => item(Value).asNum / item(Weight)).maxBy(_.asNum)
    val capacity = problem(Capacity)
    val weightLookup = problem(Items).asCol.map(item => item(Name) -> item(Weight)).toMap

    def est(x: List[Term]): Num = {
      if (x.length == weightLookup.size) Num(0)
      else {
        val weightUsed = x.map(c => c.asKvp.value.asNum * weightLookup(c.asKvp.key)).reduce(_.asNum + _.asNum)
        val weightLeft = (capacity - weightUsed).asNum
        if (weightLeft <= Num(0)) Num(0)
        else (weightLeft * bestValForWeight).asNum
      }
    }

    est
  }

  def remainingCostEstGeneratorFillWithBestLeft(problem: Term): List[Term] => Num = {
    val capacity = problem(Capacity)
    val weightLookup = problem(Items).asCol.map(item => item(Name) -> item(Weight)).toMap
    val ratioLookup = problem(Items).asCol.map(item => item(Name) -> item(Value) / item(Weight)).toMap

    def est(x: List[Term]): Num = {
      if (x.length == weightLookup.size) Num(0)
      else {
        val chosen = x.map(i => i.asKvp.key).toSet
        val bestValForWeightLeft = ratioLookup.withFilter((i, r) => !chosen.contains(i)).map((i, r) => r).maxBy(_.asNum)
        val weightUsed = x.map(c => c.asKvp.value.asNum * weightLookup(c.asKvp.key)).reduce(_.asNum + _.asNum)
        val weightLeft = (capacity - weightUsed).asNum
        if (weightLeft <= Num(0)) Num(0)
        else (weightLeft * bestValForWeightLeft).asNum
      }
    }

    est
  }

  def remainingCostEstGeneratorFillGreedy(problem: Term): List[Term] => Num = {
    val capacity = problem(Capacity)
    val limit = problem(PerItemLimit)
    val weightLookup = problem(Items).asCol.map(item => item(Name) -> item(Weight)).toMap
    val itemData = problem(Items).asCol.map(item => (item(Name), item(Value), item(Weight), item(Value) / item(Weight))).toSeq

    def est(x: List[Term]): Num = {
      if (x.length == weightLookup.size) {
        Num(0)
      } else {
        val weightUsed = x.map(c => c.asKvp.value.asNum * weightLookup(c.asKvp.key)).reduce(_.asNum + _.asNum)
        var weightLeft = (capacity - weightUsed).asNum
        if (weightLeft <= Num(0)) {
          Num(0)
        } else {
          val chosen = x.map(i => i.asKvp.key).toSet
          val leftOvers = itemData.filterNot(row => chosen.contains(row(0))).sortBy(_ (3).asNum).reverse
          if (limit.asNum.isInfPos) {
            (weightLeft * leftOvers.head(3)).asNum
          } else {
            var remCost = Num(0)
            for (item <- leftOvers) {
              for (count <- (1 until term2int(limit))) {
                remCost = (remCost + item(1).asNum).asNum
                weightLeft = (weightLeft - item(2)).asNum
                if (weightLeft <= Num(0)) {
                  return remCost
                }
              }
            }
            remCost
          }
        }
      }
    }

    est
  }

  def variableOrderingGenerator(problem: Term): Seq[Term] => Seq[Term] = {
    val score = problem(Items).asCol.map(item => item(Name) -> (item(Value) / item(Weight)).asNum).toMap

    def f(vars: Seq[Term]): Seq[Term] = {
      vars.sortBy(score(_)).reverse
    }

    f
  }

  def valueOrdering(values: Seq[Term]): Seq[Term] = values.sortBy(_.asNum).reverse
}
