package org.aiddl.common.scala.planning.state_variable.data

import scala.collection.mutable.{HashSet, HashMap}
import scala.collection.immutable

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.math.graph.Terms._
import org.aiddl.common.scala.planning.PlanningTerm._
import org.aiddl.common.scala.planning.state_variable.heuristic.CausalGraphHeuristic.Unknown

import org.aiddl.core.scala.representation.given_Conversion_Term_KeyVal
import scala.language.implicitConversions

class DomainTransitionGraphCreator(problem: CollectionTerm) extends Function {

  def apply( os: Term ): Term = {
    val domains = new HashMap[Term, HashSet[Term]]

    problem(InitialState).asCol.foreach( s => {
      domains.getOrElseUpdate(s.key, new HashSet).add(s.value)
    })

        os.asCol.foreach( o => {
            o(Preconditions).asCol.foreach( p => { domains.getOrElseUpdate(p.key, new HashSet).add(p.value) })
            o(Effects).asCol.foreach( p => { domains.getOrElseUpdate(p.key, new HashSet).add(p.value) })
        })

        SetTerm(domains.keys.map( v => {
            v :: dtg(v, domains(v), os.asCol)
        } ).toSet)
    }

    def dtg( v: Term, domain: HashSet[Term], operators: CollectionTerm ): Term = {
      var edges = new immutable.HashSet[Term]
      val labels = new HashMap[Term, HashSet[Term]]

      operators.withFilter( o => o(Effects).asCol.containsKey(v)
        && o(Preconditions).getOrElse(v, Unknown) != o(Effects)(v)  )
        .foreach( o => {
          val from = o(Preconditions).getOrElse(v, Unknown)
          val edge = Tuple(from, o(Effects)(v))
          val cond = SetTerm(o(Preconditions).asCol.filter( p => p.key != v || p.value != from ).toSet)

          labels.getOrElseUpdate(edge, new HashSet).add(cond)
          edges = edges + edge

          if ( from == Unknown ) { // Add edge from each value of the domain other than target
            domain.foreach( from => {
              if ( from != o(Effects)(v) ) {
                val edge = Tuple(from, o(Effects)(v))
                val cond = SetTerm(o(Preconditions).asCol.filter( p => p.key != v || p.value != from ).toSet)
                labels.getOrElseUpdate(edge, new HashSet).add(cond)
                edges = edges + edge
              }
            })
          }
        } )
      Tuple(
        Nodes :: SetTerm(domain.toSet),
        Edges :: SetTerm(edges),
        Labels :: SetTerm(labels.map( (k, c) => k :: SetTerm(c.toSet) ).toSet)
      )
    }
}