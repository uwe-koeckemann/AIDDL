package org.aiddl.common.scala.search

import org.aiddl.core.scala.function.{Function, Initializable, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.BoolImplicits.*
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.tools.{Logger, StopWatch}

import scala.annotation.tailrec
import scala.collection.mutable
import scala.collection.mutable.{HashMap, HashSet, PriorityQueue}

trait GenericGraphSearch[E, N] extends Verbose {
    val openList = new PriorityQueue[(Num, N)]()(Ordering.by( (x, _) => -x ))
    val closedList = new HashSet[N]
    val seenList = new HashSet[N]

    val predecessor = new HashMap[N, N]
    val distance = new HashMap[N, Int]
    val edges = new HashMap[N, E]

    var pruneFunctions: List[N => Boolean] = Nil

    var n_added = 0
    var n_opened = 0
    var n_pruned = 0
    var includePathLength = false
    var omega = Num(0.5)

    def h( n: N ): Num
    def isGoal( n: N ): Boolean
    def expand( n: N ): Seq[(E, N)]
    def propagate( n: N ): Option[N] = Some(n)

    def init( args: Iterable[N] ) = {
        openList.clear; closedList.clear; seenList.clear
        predecessor.clear; distance.clear; edges.clear
        n_added = 0; n_opened = 0; n_pruned = 0
        args.foreach( n => {
            distance.put(n, 0);
            openList.addOne((f(n), n))
            seenList.add(n) })
    }

    def step( n: N ): Num = {
        closedList.add(n)
        this.propagate(n) match {
            case None => n_pruned += 1
            case Some(n) => {
                val expansion = expand(n)
                this.n_opened += expansion.size
                logInc(1, s"Expansion size: ${expansion.size}.")
                for ((edge, dest) <- expansion if !seenList.contains(dest)) {
                    seenList.add(dest)
                    val isPruned = this.pruneFunctions.exists(f => f(dest))
                    if (isPruned) {
                        n_pruned += 1
                    }
                    else {
                        predecessor.put(dest, n)
                        edges.put(dest, edge)
                        distance.put(dest, distance(n) + 1)
                        val fVal = f(dest)
                        if ( fVal.isInfPos ) {
                            log(1, s"Node pruned because heuristic value is infinite")
                            n_pruned
                        } else {
                            log(1, s"Node score f: $fVal")
                            log(2, s"Edge: $edge")
                            log(3, s"  Path:: ${pathTo(dest).mkString(" <- ")}")
                            openList.addOne((fVal, dest))
                            n_added += 1
                        }

                    }
                }
                logDec(1, s"Added: $n_added, pruned: $n_pruned, opened: $n_opened")
            }
        }
        Num(n_added)
    }

    def g(n: N): Num = Num(distance(n))
    def f(n: N): Num =
        if ( includePathLength )
            h(n)*omega + g(n)*(Num(1.0)-omega)
        else
            h(n)

    def next: Option[(N, Boolean)] = {
        log(1, s"Next from ${openList.size} choices")
        if (openList.isEmpty) None
        else {
            val node = openList.dequeue._2
            val goalReached = isGoal(node)
            log(1, s"Selected node is goal: $goalReached")
            Some((node, goalReached))
        }
    }

    def path(n: N): List[E] = pathTo(n).reverse

    @tailrec
    final def search: Option[Seq[E]] = {
        next match {
            case None => None
            case Some((n, true)) => Some(pathTo(n).reverse)
            case Some((n, false)) => { step(n); search }
        }
    }

    private def pathTo( dest: N ): List[E] = {
        if (!edges.isDefinedAt(dest)) Nil
        else edges(dest) :: pathTo(predecessor(dest))
    }
}
