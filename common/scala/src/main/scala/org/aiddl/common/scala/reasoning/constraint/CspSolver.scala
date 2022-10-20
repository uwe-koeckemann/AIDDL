package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.search.{GenericTreeSearch, TreeSearch}
import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm.*
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.util.StopWatch

import Term.given_Conversion_Term_KeyVal
import scala.language.implicitConversions

import scala.collection.mutable

class CspSolver extends GenericTreeSearch[Term, Seq[Term]] with Initializable {
  val nil = Sym("NIL")

  var usePropagation = true
  var checkWithGroundArgsOnly = false

  var dynamicVariableOrdering: Seq[Term] => Seq[Term] = x => x
  var dynamicValueOrdering: Seq[Term] => Seq[Term] = x => x

  var staticVariableOrdering: Seq[Term] => Seq[Term] = x => x
  var staticValueOrdering: Seq[Term] => Seq[Term] = x => x


  private var vars: CollectionTerm = _
  private var domains: CollectionTerm = _
  private var cons: CollectionTerm = _

  private var propDomains: List[CollectionTerm] = Nil

  private val cMap = new mutable.HashMap[Term, Set[Term]]().withDefaultValue(Set.empty)

  override def init( csp: Term ) = {
    super.reset
    vars = ListTerm(staticVariableOrdering(csp(Variables).asList))
    domains = SetTerm(csp(Domains).asCol.map( x_d => {
      KeyVal(x_d.asKvp.key, ListTerm(staticValueOrdering(x_d.asKvp.value.asList)))
    }).toSet)
    cons = csp(Constraints).asCol

    cons.foreach( c => {
      val scope = c(0)
      scope.asTup.foreach( x => cMap.put(x, cMap(x) + c))
    })

    propDomains = List(domains)
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

  def assembleSolution( choice: List[Term] ): Option[List[Term]] = Some(choice.reverse)

  override def backtrackHook: Unit = {
    if ( usePropagation ) propDomains = propDomains.drop( propDomains.length - choice.length )
  }

  override def expand: Option[Seq[Term]] =
    val openVars = vars.filter( x => !choice.exists( a => a.key == x ) ).toSeq
    if (openVars.isEmpty) None
    else {
      val x = dynamicVariableOrdering(openVars).head
      val domain = dynamicValueOrdering(propDomains.head(x).asList)
      Some(ListTerm(domain.map( v => KeyVal(x, v)  )))
    }

  override def isConsistent: Boolean = {
    val sub = new Substitution()
    choice.foreach( a => sub.add(a.key, a.value) )
    val propagationConsistent = {
      if ( !usePropagation ) true
      else {
        var emptyDomain = false
        val newDomains = ListTerm(vars.filter(x => !choice.exists(y => x == y.key)).map(x => {
          val newDomain = ListTerm(propDomains.head(x).asCol.filter(v => {
            val sub_x = new Substitution()
            sub_x.add(x, v)
            cMap(x).intersect(cMap(choice.head.key)).forall(c => {
              val args = (c(0) \ sub) \ sub_x
              val pCon = c(1)
              if (args.isGround) {
                try {
                  pCon(args).boolVal
                } catch {
                  case _ => true
                }
              } else true
            })
          }).toVector)
          if (newDomain.length == 0) emptyDomain = true
          KeyVal(x, newDomain)
        }).toList)
        if (!emptyDomain) {
          propDomains = newDomains :: propDomains
        }
        !emptyDomain
      }
    }
    val con = propagationConsistent && cons.forall( c => {
      val args = c(0)\sub
      val pCon = c(1)
      try {
        if ( checkWithGroundArgsOnly && !args.isGround )
          true
        else
          pCon(args).boolVal
      } catch {
        case _ => true
      }
    })

    con
  }
}
