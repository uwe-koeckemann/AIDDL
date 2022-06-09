package org.aiddl.common.scala.optimization.combinatorial

import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm.*
import org.aiddl.common.scala.reasoning.constraint.CspSolver
import org.aiddl.common.scala.search.TreeSearch
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.BoolImplicits.term2Boolean
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2CollectionTerm
import org.aiddl.core.scala.representation.TermImplicits.term2Num
import org.aiddl.core.scala.representation.TermImplicits.term2KeyVal

class BranchAndBound extends CspSolver {
  private var costFunctions: CollectionTerm = _
  private var costType: Sym = _
  private var remainingCostEst: List[Term] => Num = _ => Num(0)

  override def init( csp: Term ) = {
    costType = csp(Sym("cost"))(0).asSym
    costFunctions = csp(Sym("cost"))(1).asCol

    super.init(csp)

    costType match {
      case Sym("min") => this.best = InfPos()
      case Sym("max") => this.best = InfNeg()
    }
  }

  override def cost: Option[Num] = {
    val sub = new Substitution()
    choice.foreach( a => sub.add(a.key, a.value) )
    val cs = costFunctions.map( c => {
      val args = c(0)\sub
      val pCon = c(1)
      if ( args.isGround ) pCon(args) else NaN()
    }).reduce(_ + _).asNum

    cs match {
      case NaN() => None
      case x: Num => Some(x)
    }
  }

  def setRemainingCostFunction( f: List[Term] => Num ) = {
    this.remainingCostEst = f
    this.allowEarlyCostPruning = true
  }

  override def costAcceptable(c: Num): Boolean =  {
    costType match {
      case Sym("min") => c + remainingCostEst(choice) < this.best
      case Sym("max") => c + remainingCostEst(choice) > this.best
    }
  }
}