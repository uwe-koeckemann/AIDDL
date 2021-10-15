package org.aiddl.common.scala.planning.spiderplan.resolvers

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.spiderplan.{ResolverGenerator, ResolverIterator, ResolverSequenceIterator}
import org.aiddl.common.scala.planning.spiderplan.SpiderPlan.*
import org.aiddl.common.scala.planning.spiderplan.resolvers.ReusableResourceScheduler
import org.aiddl.common.scala.reasoning.resource.{FlexibilityOrdering, LinearMcsSampler}
import org.aiddl.core.scala.function.{Function, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.TermImplicits.term2KeyVal
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2CollectionTerm
import org.aiddl.core.scala.tools.Logger

import scala.collection.{immutable, mutable}

class OpenGoalResolver extends ResolverGenerator with Verbose {
  override val targets: List[Sym] = List(OpenGoal)
  this.setVerbose(1)

  def apply( cdb: Term ): ResolverIterator = {
    val openGoals = cdb.getOrElse(OpenGoal, SetTerm.empty).filter( g => g(0).isInstanceOf[Var] )
    val statements = cdb.getOrElse(Statement, SetTerm.empty).asCol
    val operators = cdb.getOrElse(Operator, SetTerm.empty).asCol
    val propValues = cdb.getOrElse(PropagatedValue, SetTerm.empty)

    if ( openGoals.isEmpty ) {
      new ResolverSequenceIterator(true, List(SetTerm.empty))
    } else {
      val goal: Term = openGoals.head // TODO: Apply goal/variable ordering
      this.logInc(1, s"Selected goal: $goal")
      val statementResolvers: Seq[Term] = statements.flatMap( (statement: Term) => {
        val sInt = goal(0).unify(statement(0))
        val sKvp = statement(1).unify(goal(1))
        val sCom = sInt.flatMap( _ + sKvp )
        log(1, s"$goal <-> $statement => $sInt + $sKvp = $sCom")
        sCom match {
          case Some(sub) => {
            this.log(1, s"Substitution resolver: $sub")
            List(ListTerm(List(Tuple(Substitute, sub.asTerm))))
          }
          case _ => List()
        }
      }).toSeq
      val opResolvers: Seq[Term] = operators.flatMap( o => {
        o(Effects).flatMap( (eff: Term) => {
          val sInt = goal(0).unify(eff(0))
          val sKvp = eff(1).unify(goal(1))
          val sCom = sInt.flatMap( _ + sKvp )
          log(1, s"$goal <-> $eff => $sInt + $sKvp = $sCom")
          sCom match {
            case Some(sub) => {
              val oSub = o\sub

              val oConsRes: Seq[Term] = oSub(Constraints).map( c => {
                c match {
                  case KeyVal(cType, cs) => Tuple(AddAll, cType, cs)
                  case _ => throw new IllegalArgumentException(s"Unsupported constraint entry $c.\nPLease use format type:collection")
                }
              }).toSeq

              val r = List(ListTerm(oConsRes ++ List(
                Tuple(AddAll, Statement, oSub(Effects)),
                Tuple(AddAll, OpenGoal, oSub(Preconditions)),
                Tuple(Substitute, sub.asTerm)
              )))
              this.log(1, s"Operator resolver: $r")

              r
            }
          case _ => List()
        }
        })
      }).toSeq
      Logger.--
      // TODO: Apply value ordering
      val resolverList = statementResolvers ++ opResolvers
      new ResolverSequenceIterator(false, resolverList)
    }
  }
}
