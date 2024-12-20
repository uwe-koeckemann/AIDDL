package org.aiddl.common.scala.planning.state_variable.heuristic

import org.aiddl.common.scala.math.graph.GraphType.Directed
import scala.collection.{immutable, mutable}
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
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.planning.PlanningTerm.*
import org.aiddl.common.scala.planning.state_variable.ReachableOperatorEnumerator
import org.aiddl.common.scala.planning.state_variable.data.CausalGraphCreator
import org.aiddl.common.scala.planning.state_variable.data.DomainTransitionGraphCreator
import org.aiddl.common.scala.math.graph.{AdjacencyListGraph, Graph, Graph2Dot}
import org.aiddl.common.scala.math.graph.GraphType.Directed
import org.aiddl.common.scala.math.graph.Terms.*
import org.aiddl.core.scala.util.logger.Logger
import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_KeyVal

import scala.language.implicitConversions

object CausalGraphHeuristic {
    val Unknown: Sym = Sym("#unknown#")
}

class CausalGraphHeuristic extends Function with InterfaceImplementation with Initializable with Heuristic {
    import CausalGraphHeuristic.Unknown
    val interfaceUri: Sym = Sym("org.aiddl.common.planning.state-variable.heuristic");

    val createCG = new CausalGraphCreator

    var cg: Graph = _
    var dtgs: immutable.Map[Term, Graph] = _
    var g: SetTerm = _

    val costCache = new mutable.HashMap[(Term, Term, Term), mutable.Map[Term, Num]]
    
    def init( args: Term ): Unit = {
        val createDTGs = new DomainTransitionGraphCreator(args.asCol)

        costCache.clear()
        //val f = new ReachableOperatorEnumerator
        //val actions = f(args(Operators).asSet, args(InitialState).asSet)
        cg = AdjacencyListGraph(createCG(args(Operators)))
        dtgs = createDTGs(args(Operators)).asSet.map( e => e.key -> AdjacencyListGraph(e.value) ).toMap
        g = args(Goal).asSet
    }

    /**
     * Export causal graph and domain transition graphs to DOT files for inspection
     */
    def graphs2dot(): Unit = {
        val g2d = new Graph2Dot(Directed)
        g2d.graph2file(cg, "causal-graph.dot")

        for (x <- dtgs.keys) {
            g2d.graph2file(dtgs(x), s"${x.toString.replace("(", "_").replace(")", "_")}.dot")
        }
    }

    def apply( s: SetTerm ): Num = {
        val finalVal = g.foldLeft(Num(0))( (c, goal) => {
            if ( c.isInfPos ) {
                InfPos()
            } else {
                val cVal =  s.getOrElse(goal.key, Unknown)
                val gCost = cost(s, goal.key, cVal, goal.value)
                c + gCost
            }
        })
        finalVal
    }

    def repeatSpace(n: Int): String = {
        var s: String = ""
        (0 to n).foreach(i => {
            s = s + "  "
        })
        s
    }

    def apply( args: Term ): Num =
        this(args.asSet)

    def cost( s: SetTerm, x: Term, v_current: Term, v_target: Term ): Num = {
        if v_current == v_target
        then Num(0)
        else {
            val context = (s, x, v_current)
            val cache = costCache.getOrElseUpdate(context, new mutable.HashMap[Term, Num]())
            cache.get(v_target) match {
                case Some(cost) => {
                    cost.asNum
                }
                case None => {
                    val localState = this.getLocalState(s, x)
                    val localStateMap = new HashMap[Term, SetTerm]
                    localStateMap.put(v_current, localState)
                    localStateMap.put(Unknown, localState)
                    val dtg = dtgs.get(x) match {
                        case None => {
                            InfPos()
                        }
                        case Some(dtg) => {
                            val unreached = new mutable.HashSet[Term]
                            dtg.nodes.foreach( n => cache.put(n, InfPos()) )

                            unreached.addAll(dtg.nodes)
                            cache.put(v_current, Num(0))
                            cache.put(Unknown, Num(0))

                            var next: Option[Term] = None
                            while ( { next = chooseNext(cache, unreached); next.isDefined } ) {
                                val d_i = next.get                       
                                unreached.remove(d_i)
                                val localState_d_i = localStateMap(d_i)
                                dtg.outNeighbors(d_i).foreach( d_j => {
                                    dtg.label(d_i, d_j) match { 
                                        case None => {}
                                        case Some(l) => {
                                            l.asCol.foreach( conds => {
                                                val applicableConditions = conds.asCol.filter( x => localState.containsKey(x.key))
                                                val transCost = applicableConditions.foldLeft(Num(1))( (c, cond) => {
                                                    if ( c.isInfPos ) c
                                                    else localState_d_i.get(cond.key) match {
                                                        case Some(e_cur) => c + {
                                                            val subCost = this.cost(s, cond.key, e_cur, cond.value)
                                                            subCost
                                                        }
                                                        case None => Num(0) // c // Num(0) before but this seemed wrong
                                                    }
                                                })
                                                if ( cache(d_i) + transCost < cache(d_j) ) {
                                                    cache.put(d_j, cache(d_i) + transCost)
                                                    val localState_d_j = SetTerm({
                                                        localState_d_i.filter( sva => !conds.asCol.containsKey(sva.key) ) ++ conds.asSet.set
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

    private def chooseNext( cache: Map[Term, Num], unreached: mutable.HashSet[Term] ): Option[Term] = {
        unreached.minByOption(cache(_))  match { 
            case Some(v) if (cache(v).isInfPos) => None
            case e => e
        }
    }

    private def getLocalState( s: SetTerm, variable: Term ): SetTerm =
        SetTerm(cg.inNeighbors(variable).map( p => KeyVal(p, s.getOrElse(p, Unknown)) ).toSet)
}