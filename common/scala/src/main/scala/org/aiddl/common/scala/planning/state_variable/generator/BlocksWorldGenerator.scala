package org.aiddl.common.scala.planning.state_variable.generator

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.*

import scala.util.Random

class BlocksWorldGenerator(rng: Random = Random(0)) extends Function {

  private def toState(stack: List[Term]): List[Term] =
    stack match
      case ::(head, Nil) => List(KeyVal(Tuple(Sym("on"), head), Sym("free")))
      case ::(head, next) => KeyVal(Tuple(Sym("on"), head), next.head) :: toState(next)
      case Nil => Nil

  def stackListToState(stack: List[Term]): Term = {
    val notOnTable = stack.tail.map(x => KeyVal(Tuple(Sym("on-table"), x), Bool(false)))
    SetTerm(
      (KeyVal(Sym("holding"), Sym("nothing"))
        :: KeyVal(Tuple(Sym("on-table"), stack.head), Bool(true))
        :: toState(stack) ++ notOnTable).toSet)
  }

  def stackListToGoal(stack: List[Term]): Term = {
    SetTerm(
      (KeyVal(Tuple(Sym("on-table"), stack.head), Bool(true))
        :: toState(stack)).toSet)
  }

  override def apply(x: Term): Term = {
    val numBlocks = x(Sym("num-blocks")).intoInt
    val numStacksInit = x.get(Sym("num-stacks-init"))
    val numStacksGoal = x.get(Sym("num-stacks-goal"))

    val initStack: List[Term] =
      rng.shuffle((1 to numBlocks).map( x => Sym(s"b$x")).toList)
    val goalStack: List[Term] =
      rng.shuffle((1 to numBlocks).map(x => Sym(s"b$x")).toList)

    Tuple(
      stackListToState(initStack),
      stackListToGoal(goalStack),
    )
  }
}
