package org.aiddl.common.scala.planning.task_network

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.function.Configurable
import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.planning.state_variable.OperatorStateEnumerator
import org.aiddl.common.scala.planning.state_variable.heuristic.SumCostHeuristic
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.state_variable.{ApplicableFunction, StateTransition}

import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_SetTerm
import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_Tuple

import scala.language.implicitConversions

class TotalOrderForwardDecomposition extends Function with Initializable with Verbose {
  var os: SetTerm = _
  var ms: SetTerm = _
  var pts: Set[Term] = _

  val applicable = new ApplicableFunction
  val transition = new StateTransition
  val opEnum = new OperatorStateEnumerator

  def init( args: Term ): Unit = {
    this.os = args(Operators)
    this.ms = args(Methods)
    this.pts = os.map( o => o(Name)(0) ).toSet
  }

  def apply( s: SetTerm, ots: ListTerm ): Option[List[Term]] = {
    logger.info(s"Open Tasks: $ots")
    if ( ots.isEmpty ) Some(Nil)
    else {
      var sol: Option[List[Term]] = None
      val t = ots.head
      if ( this.pts.contains(t(0)) ) {
        logger.fine(s"State: $s")
        logger.info(s"Primitive task: $t")
        logger.depth += 1
        this.os.find( (a:Term) => {
          a(Name) unify t match {
            case None => false
            case Some(sub) if applicable(a\sub, s).boolVal => {
              val aSub = a\sub
              val sNext = transition(aSub, s)
              val w = ListTerm(ots.tail.map( x => x \ sub ))
              logger.info(s"Trying: ${aSub(Name)}")
              logger.info(s"  Open: $w")
              val r = this.apply(sNext, w)
              r match {
                case Some(pi) => {
                  val sel = aSub(Name)
                  sol = Some(sel :: pi)
                  true
                }
                case None => false
              }
            }
            case _ => {
              logger.info(s"  No match: $t <-> ${a(Name)}")
              false
            }
          }
        })
        sol
      } else {
        logger.info(s"State: $s")
        logger.info(s"Non-primitive task: $t")
        logger.depth += 1

        this.ms.find( m => {
          m(Task) unify t match {
            case Some(sub) => {
              opEnum(s, SetTerm(m \ sub)).exists(mg => {
                t unify mg(Task) match {
                  case Some(sub) => {
                    val w = ListTerm(mg(SubTasks).asList ++ ots.tail.map(_ \ sub))
                    logger.info(s"Trying: ${mg(Name)}")
                    logger.info(s"  Open: $w")
                    this (s, w) match {
                      case Some(pi) => {
                        sol = Some(pi); true
                      }
                      case None => false
                    }
                  }
                  case None => false
                }
              })
            }
            case None => false
          }
        })
      }
      logger.depth -= 1
      logger.info(s"Result: $sol")
      sol
    }
  }

  def apply( args: Term ): Term = this(args(InitialState), args(OpenTasks).asList) match {
    case Some(sol) => ListTerm(sol)
    case None => NIL
  }
}
