package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.search.GenericTreeSearch
import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm.*
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.util.StopWatch

import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_KeyVal
import scala.language.implicitConversions

import scala.collection.mutable

object CspSolver {
  def checkConstraint(constraint: Term, args: Term): Boolean = {
    if constraint.isInstanceOf[CollectionTerm]
    then {
      val r = if args.isGround
      then constraint.asCol.contains(args)
      else constraint.asCol.exists(x => args.unifiable(x))
      r
    } else {
      try {
        constraint(args).boolVal
      } catch {
        case _ => true
      }
    }
  }
}

class CspSolver extends GenericTreeSearch[Term, Seq[Term]] with Initializable {
  val nil: Sym = Sym("NIL")

  var propagationFunction: Option[PropagationFunction] = Some(new ForwardChecking)
  var checkWithGroundArgsOnly = false

  var dynamicVariableOrdering: Seq[Term] => Seq[Term] = x => x
  var dynamicValueOrdering: Seq[Term] => Seq[Term] = x => x

  var staticVariableOrdering: Seq[Term] => Seq[Term] = x => x
  var staticValueOrdering: Seq[Term] => Seq[Term] = x => x


  private var csp: ConstraintSatisfactionProblem = _

  private var propDomains: List[Map[Term, Seq[Term]]] = Nil

  def init(csp: ConstraintSatisfactionProblem): Unit = {
    this.csp = csp
    super.reset
    propagationFunction.foreach(_.init(this.csp))
    csp.variables = staticVariableOrdering(csp.variables)
    csp.domains = csp.domains.map((x, d) => {
      x -> ListTerm(staticValueOrdering(d))
    })

    propDomains = List(csp.domains)
  }

  override def init( cspTerm: Term ): Unit = {
    this.init(ConstraintSatisfactionProblem.fromTerm(cspTerm))
  }

  def apply( args: Term ): Term =
    args match {
      case Tuple(Sym("search")) => search match { case Some(c) => ListTerm(c) case None => NIL }
      case Tuple(Sym("optimal")) => optimal match { case Some(c) => ListTerm(c) case None => NIL }
      case _ => {
        init(args)
        search match {
          case Some(s) => ListTerm(s)
          case None => NIL
        }
      }
    }

  def assembleSolution( choice: List[Term] ): Option[List[Term]] =
    Some(choice.reverse)

  override def backtrackHook: Unit = {
    if ( propagationFunction.isDefined ) propDomains = propDomains.drop( propDomains.length - choice.length )
  }

  override def expand: Option[Seq[Term]] =
    val openVars = csp.variables.filter( x => !choice.exists( a => a.key == x ) )
    if (openVars.isEmpty) {
      None
    } else {
      val x = dynamicVariableOrdering(openVars).head
      val domain = dynamicValueOrdering(this.currentDomains(x))
      Some(ListTerm(domain.map( v => KeyVal(x, v))))
    }

  override def isConsistent: Boolean = {
    val sub = new Substitution()
    choice.foreach( a => sub.add(a.key, a.value) )
    val propagationConsistent = {
      propagationFunction match
        case Some(fPropagate) => {
          fPropagate.propagate(choice, this.currentDomains) match
            case Some(newDomains) => {
              propDomains = newDomains :: propDomains
              true
            }
            case None => false
        }
        case None => true
    }
    val con = propagationConsistent && csp.constraints.forall( c => {
      val args = c.scope\sub
      if checkWithGroundArgsOnly && !args.isGround
      then true
      else c.satisfiedBy(sub)
    })
    con
  }

  private def currentDomains: Map[Term, Seq[Term]] =
    if propagationFunction.isDefined
    then
      if propDomains.isEmpty
      then csp.domains
      else propDomains.head
    else csp.domains
}
