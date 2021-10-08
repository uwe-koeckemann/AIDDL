package org.aiddl.common.scala.planning.state_variable.heuristic

import scala.collection.immutable
import scala.collection.mutable.Map
import scala.collection.mutable.HashMap
import scala.collection.mutable.HashSet
import scala.annotation.tailrec

import org.aiddl.core.scala.function.InterfaceImplementation
import org.aiddl.core.scala.function.Initializable

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.function.Configurable
import org.aiddl.core.scala.function.Verbose

import org.aiddl.core.scala.container.Container

import org.aiddl.core.scala.representation._

import org.aiddl.core.scala.representation.TermImplicits._
import org.aiddl.core.scala.representation.BoolImplicits._
import org.aiddl.common.scala.planning.PlanningTerm._
import org.aiddl.common.scala.planning.state_variable.ReachableOperatorEnumerator
import org.aiddl.common.scala.planning.state_variable.data.CausalGraphCreator
import org.aiddl.common.scala.planning.state_variable.data.DomainTransitionGraphCreator
import org.aiddl.common.scala.math.graph.{AdjacencyListGraph, Graph}
import org.aiddl.common.scala.math.graph.Terms._

import org.aiddl.core.scala.representation.TermImplicits._
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2SetTerm

object CausalGraphHeuristic {
    val Unknown = Sym("#unknown#")
}

class CausalGraphHeuristic extends Function with InterfaceImplementation with Initializable {
    import CausalGraphHeuristic.Unknown
    val interfaceUri = Sym("org.aiddl.common.planning.state-variable.heuristic");

    val createCG = new CausalGraphCreator
    val createDTGs = new DomainTransitionGraphCreator

    var cg: Graph = _
    var dtgs: immutable.Map[Term, Graph] = _
    var g: SetTerm = _

    val costCache = new HashMap[(Term, Term, Term), Map[Term, Num]]
    
    def init( args: Term ) = {
        val f = new ReachableOperatorEnumerator
        val actions = f(args(Operators), args(InitialState))

        cg = AdjacencyListGraph(createCG(actions))
        dtgs = createDTGs(actions).asSet.map( e => e.key -> AdjacencyListGraph(e.value) ).toMap
        
        g = args(Goal)
    } 

    def apply( s: SetTerm ): Num = {
        g.foldLeft(Num(0))( (c, goal) => {
            if ( c == InfPos() ) InfPos()
            else c + cost(s, goal.key, s.getOrElse(goal.key, Unknown), goal.value)
        })        
    }

    def apply( args: Term ): Term = args match {
        case s: SetTerm => this(s)
        case _ => ???
    }

    def cost( s: SetTerm, x: Term, v_current: Term, v_target: Term ): Num = {
        if ( v_current == v_target ) Num(0) else {
            val context = (s, x, v_current)
            val cache = costCache.getOrElseUpdate(context, new HashMap[Term, Num]())
            cache.get(v_target) match {
                case Some(cost) => cost 
                case None => {
                    val localState = this.getLocalState(s, x)
                    val localStateMap = new HashMap[Term, SetTerm]
                    localStateMap.put(v_current, localState)
                    localStateMap.put(Unknown, localState)
                    val dtg = dtgs.get(x) match {
                        case None => InfPos()
                        case Some(dtg) => {
                            val unreached = new HashSet[Term]
                            dtg.nodes.foreach( n => cache.put(n, InfPos()) )
                            unreached.addAll(dtg.nodes)
                            cache.put(v_current, 0)
                            cache.put(Unknown, 0)
                    
                            var next: Option[Term] = None
                            while ( { next = chooseNext(cache, unreached); next != None } ) {
                                val d_i = next.get                       
                                unreached.remove(d_i)
                                val localState_d_i = localStateMap(d_i) 
                                dtg.outNeighbors(d_i).foreach( d_j => {
                                    dtg.label(d_i, d_j) match { 
                                        case None => {}
                                        case Some(l) => {
                                            l.foreach( conds => {
                                                val transCost = conds.foldLeft(Num(1))( (c, cond) => {
                                                    if ( c == InfPos() ) c 
                                                    else localState_d_i.get(cond.key) match {
                                                        case Some(e_cur) => c + this.cost(s, cond.key, e_cur, cond.value)
                                                        case None => Num(0)
                                                    }
                                                }) 
                                                if ( cache(d_i) + transCost < cache(d_j) ) {
                                                    cache.put(d_j, cache(d_i) + transCost)
                                                    val localState_d_j = SetTerm({
                                                        localState_d_i.filter( sva => !conds.containsKey(sva.key) ) ++ conds
                                                    }.toSet)
                                                    localStateMap.put(d_j, localState_d_j)
                                                }
                                            })
                                        }
                                    }
                                })
                            }
                        }
                    }
                    cache.getOrElse(v_target, InfPos())
                }
            }
        }
    }

    private def chooseNext( cache: Map[Term, Num], unreached: HashSet[Term] ): Option[Term] = {
        unreached.minByOption(cache(_))  match { 
            case Some(v) if (cache(v) == InfPos()) => None
            case e => e
        }
    }

    private def getLocalState( s: SetTerm, variable: Term ): SetTerm =
        SetTerm(cg.inNeighbors(variable).map( p => p :: s.getOrElse(p, Unknown) ).toSet)
}