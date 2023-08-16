package org.aiddl.example.learning_agent_scala.function

import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.{CollectionTerm, KeyVal, ListTerm, SetTerm, Sym, Term, Tuple}
import org.aiddl.common.scala.planning.PlanningTerm

object OperatorCreator extends Function with Verbose {

  override def apply(tree: Term): Term = {
    val oNew = this
      .extract(tree, ListTerm.empty, null)
      .sortWith((a, b) => a.toString.compareTo(b.toString) < 0)
    SetTerm(oNew.toSet)
  }

  private def isCollectionOfSets(s: CollectionTerm): Boolean =
    s.forall( t => t.isInstanceOf[SetTerm] )

  private def extract(tree: Term, preCurrent: ListTerm, actionName: Term): List[Term] = {
    var O: List[Term] = Nil
    if (!tree.isInstanceOf[ListTerm]) {
      val eff = tree.asInstanceOf[ListTerm]

      if (this.isCollectionOfSets(eff)) {
        for (eff_inside <- eff) {
          val actual_eff = eff_inside.asSet
          if (actual_eff.nonEmpty) {
            val preCol = preCurrent.asSet
            if (actionName != null) {
              val namePair = KeyVal(PlanningTerm.Name, actionName)
              val prePair = KeyVal(PlanningTerm.Preconditions, preCol)
              val effPair = KeyVal(PlanningTerm.Effects, actual_eff)
              O = Tuple(namePair, prePair, effPair) :: O
            }
          }
        }
      } else if (eff.size > 0) {
        val name = actionName
        if (actionName != null) {
          val namePair = KeyVal(PlanningTerm.Name, name)
          val prePair = KeyVal(PlanningTerm.Preconditions, preCurrent.asSet)
          val effPair = KeyVal(PlanningTerm.Effects, eff)
          O = Tuple(namePair, prePair, effPair) :: O
        }
      }
    } else {
      val decisions = tree.asList
      for (decision <- decisions) {
        var usedActionName = actionName
        val condition = decision.asTup
        val subTree = decision(1)
        val preAtom = condition(1)
        val preValue = condition(2)
        val preNew: ListTerm = if (preAtom.equals(Sym("Action"))) {
            usedActionName = preValue
            preCurrent
          } else {
            preCurrent.add(KeyVal(preAtom, preValue))
          }
          O = O.prependedAll(extract(subTree, preNew, actionName))
        }
      }
    O
  }
}
