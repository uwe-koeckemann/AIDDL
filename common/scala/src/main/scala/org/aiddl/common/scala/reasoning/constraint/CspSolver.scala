package org.aiddl.common.scala.reasoning.constraint

import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.search.TreeSearch
import org.aiddl.core.scala.representation.BoolImplicits.term2Boolean
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2CollectionTerm
import org.aiddl.common.scala.reasoning.constraint.ConstraintTerm.*

import scala.collection.mutable

class CspSolver extends TreeSearch {
  var usePropagation = true

  private var vars: CollectionTerm = _
  private var doms: CollectionTerm = _
  private var cons: CollectionTerm = _

  private var propDomains: List[CollectionTerm] = Nil

  private val cMap = new mutable.HashMap[Term, Set[Term]]().withDefaultValue(Set.empty)

  override def init( csp: Term ) = {
    vars = csp(Variables)
    doms = csp(Domains)
    cons = csp(Constraints)

    cons.foreach( c => {
      val scope = c(0)
      scope.asTup.foreach( x => cMap.put(x, cMap(x) + c))
    })

    propDomains = List(doms)

    super.init(csp)
  }

  override def backtrackHook: Unit = {
    println("-----> bt hook")
    println(propDomains)
    if ( usePropagation ) propDomains = propDomains.drop( propDomains.length - choice.length )
    println(propDomains)
  }

  override def expand: Option[Seq[Term]] =
    vars.find( x => !choice.exists( a => a.key == x ) )
        .flatMap( x => Some(ListTerm(propDomains.head(x).map( v => KeyVal(x, v)  ).toSeq)))

  override def isConsistent: Boolean = {
    val sub = new Substitution()
    choice.foreach( a => sub.add(a.key, a.value) )

    val propagationConsistent = {
      if ( !usePropagation ) true
      else {
        println(choice)
        println(propDomains.head)
        println(propDomains)
        var emptyDomain = false
        val newDomains = ListTerm(vars.filter(x => !choice.exists(y => x == y.key)).map(x => {
          val newDomain = ListTerm(propDomains.head(x).filter(v => {
            val sub_x = new Substitution()
            sub_x.add(x, v)
            cMap(x).intersect(cMap(choice.head.key)).forall(c => {
              val args = (c(0) \ sub) \ sub_x
              val pCon = c(1)
              if (args.isGround) pCon(args) else true
            })
          }).toVector)
          if (newDomain.length == 0) emptyDomain = true
          KeyVal(x, newDomain)
        }).toList)
        println(s"New domains: $newDomains")
        if (!emptyDomain) {
          propDomains = newDomains :: propDomains
        }
        !emptyDomain
      }
    }

    propagationConsistent && cons.forall( c => {
      val args = c(0)\sub
      val pCon = c(1)
      try {
        pCon(args)
      } catch {
        case _ => true
      }
    })
  }
}
