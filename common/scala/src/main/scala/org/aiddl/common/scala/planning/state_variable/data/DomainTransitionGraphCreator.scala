package org.aiddl.common.scala.planning.state_variable.data

import scala.collection.mutable.{HashSet, HashMap}
import scala.collection.immutable

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.math.graph.Terms._
import org.aiddl.common.scala.planning.PlanningTerm._
import org.aiddl.common.scala.planning.state_variable.heuristic.CausalGraphHeuristic.Unknown

import org.aiddl.core.scala.representation.TermImplicits._
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2CollectionTerm

class DomainTransitionGraphCreator extends Function {

    def apply( os: Term ): Term = {
        val domains = new HashMap[Term, HashSet[Term]]

        os.foreach( o => {
            o(Preconditions).foreach( p => { domains.getOrElseUpdate(p.key, new HashSet).add(p.value) })
            o(Effects).foreach( p => { domains.getOrElseUpdate(p.key, new HashSet).add(p.value) })
        })

        SetTerm(domains.keys.map( v => {
            v :: dtg(v, domains(v), os)
        } ).toSet)
    }

    def dtg( v: Term, domain: HashSet[Term], operators: CollectionTerm ): Term = {
        var edges = new immutable.HashSet[Term]
        val labels = new HashMap[Term, HashSet[Term]]

        operators.withFilter( o => o(Effects).containsKey(v) 
                            && o(Preconditions).getOrElse(v, Unknown) != o(Effects)(v)  )
           .foreach( o => {
                val from = o(Preconditions).getOrElse(v, Unknown)
                val edge = Tuple(from, o(Effects)(v))
                val cond = SetTerm(o(Preconditions).filter( p => p.key != v || p.value != from ).toSet)

                labels.getOrElseUpdate(edge, new HashSet).add(cond)
                edges = edges + edge
           } )
        Tuple(
            Nodes :: SetTerm(domain.toSet),
            Edges :: SetTerm(edges),
            Labels :: SetTerm(labels.map( (k, c) => k :: SetTerm(c.toSet) ).toSet)
        )
    }
}