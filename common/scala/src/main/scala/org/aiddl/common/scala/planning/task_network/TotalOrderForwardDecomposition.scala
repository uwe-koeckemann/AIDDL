package org.aiddl.common.scala.planning.task_network

import org.aiddl.core.scala.util.Logger
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.function.Configurable
import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.container.Container
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.search.GraphSearch
import org.aiddl.common.scala.planning.state_variable.OperatorStateEnumerator
import org.aiddl.common.scala.planning.state_variable.heuristic.SumCostHeuristic
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.state_variable.{ApplicableFunction, StateTransition}
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2SetTerm
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2Tuple
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.BoolImplicits.bool2Boolean
import org.aiddl.core.scala.representation.BoolImplicits.term2Boolean


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
    log(1, s"Open Tasks: $ots")
    if ( ots.isEmpty ) Some(Nil)
    else {
      var sol: Option[List[Term]] = None
      val t = ots.head
      if ( this.pts.contains(t(0)) ) {
        log(1, s"State: $s")
        log(1, s"Primitive task: $t")
        Logger.++
        this.os.find( (a:Term) => {
          a(Name) unify t match {
            case None => false
            case Some(sub) if applicable(a\sub, s) => {
              val aSub = a\sub
              val sNext = transition(aSub, s)
              val w = ListTerm(ots.tail.map( _ \ sub ))
              log(1, s"Trying: ${aSub(Name)}")
              log(1, s"  Open: $w")
              val r = this(sNext, w)
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
              log(1, s"  No match: $t <-> ${a(Name)}")
              false
            }
          }
        })
        sol
      } else {
        log(1, s"State: $s")
        log(1, s"Non-primitive task: $t")
        Logger.++
        this.ms.find( m => {
          m(Task) unify t match {
            case Some(sub) => {
              opEnum(s, SetTerm(m \ sub)).find( mg => {
                t unify mg(Task) match {
                  case Some(sub) => {
                    val w = ListTerm(mg(SubTasks).asList ++ ots.tail.map( _ \ sub ))
                    log(1, s"Trying: ${mg(Name)}")
                    log(1, s"  Open: $w")
                    this(s, w) match {
                      case Some(pi) => {sol = Some(pi); true}
                        case None => false
                    }
                  }
                  case None => false
                }
              } ) != None
            }
            case None => false
          }
        })
      }
      Logger.--
      log(1, s"Result: $sol")
      sol
    }
  }

  def apply( args: Term ): Term = this(args(InitialState), args(OpenTasks).asList) match {
    case Some(sol) => ListTerm(sol)
    case None => NIL
  }
}
