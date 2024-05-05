package org.aiddl.common.scala.planning.state_variable.data

import scala.collection.mutable.{HashMap, HashSet}
import scala.collection.{immutable, mutable}
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.math.graph.Terms.*
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.state_variable.heuristic.CausalGraphHeuristic.Unknown
import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_KeyVal

import scala.language.implicitConversions


class DomainTransitionGraphCreator(problem: CollectionTerm) extends Function {

  def apply( os: Term ): Term = {
    val domains = new mutable.HashMap[Term, mutable.HashSet[Term]]

    problem(InitialState).asCol.foreach( s => {
      domains.getOrElseUpdate(s.key, new mutable.HashSet).add(s.value)
    })

    os.asCol.foreach( o => {
        o(Preconditions).asCol.foreach( p => { domains.getOrElseUpdate(p.key, new mutable.HashSet).add(p.value) })
        o(Effects).asCol.foreach( p => { domains.getOrElseUpdate(p.key, new mutable.HashSet).add(p.value) })
    })

    SetTerm(domains.keys.map( v => {
        KeyVal(v, dtg(v, domains(v), os.asCol))
    } ).toSet)
  }

  def dtg(v: Term, domain: mutable.HashSet[Term], operators: CollectionTerm ): Term = {
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
      KeyVal(Nodes, SetTerm(domain.toSet)),
      KeyVal(Edges, SetTerm(edges)),
      KeyVal(Labels, SetTerm(labels.map( (k, c) => KeyVal(k, SetTerm(c.toSet)) ).toSet))
    )
  }
}