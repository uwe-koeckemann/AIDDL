package org.aiddl.common.scala.search

import org.aiddl.core.scala.function.{Function, Initializable, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.StopWatch
import org.aiddl.core.scala.util.logger.Logger

import scala.annotation.tailrec
import scala.collection.mutable
import scala.collection.mutable.{HashMap, HashSet, PriorityQueue}

trait GenericGraphSearch[E, N] extends Verbose {
    val openList = new PriorityQueue[(Num, N)]()(Ordering.by( (x, _) => -x ))
    val closedList = new HashSet[N]
    val seenList = new HashSet[N]
    val prunedList = new HashSet[N]
    val prunedReason = new HashMap[N, String]
    var goalList: List[N] = Nil

    val predecessor = new HashMap[N, N]
    val distance = new HashMap[N, Int]
    val edges = new HashMap[N, E]

    val tDiscovery = new HashMap[N, Int]
    val tClosed = new HashMap[N, Int]

    //var solutionPath: Option[Seq[E]] = None

    var pruneFunctions: List[N => Boolean] = Nil

    var n_added = 0
    var n_opened = 0
    var n_closed = 0
    var n_pruned = 0

    var includePathLength = false
    var pruneOnInfiniteHeuristicValue = true
    var omega = Num(0.5)

    private var tNextDiscovery: Int = 0
    private var tNextClosed: Int = 0

    def getDiscoveryTime: Int = {
        tNextDiscovery += 1
        tNextDiscovery
    }

    def getClosedTime: Int = {
        tNextClosed += 1
        tNextClosed
    }

    def h( n: N ): Num
    def isGoal( n: N ): Boolean
    def expand( n: N ): Seq[(E, N)]

    def addPrunedReason(n: N, reason: String) = {
        this.prunedReason.getOrElseUpdate(n, reason)
    }


    /**
     * Propagate node n. This may lead to pruning, no change, or a forced move in the search space.
     * @param n
     * @return <code>None</code> if propagation finds node inconsistent, <code>Some(n, None)</code> if no change was
     *         made to <code>n</code>, <code>Some(n', e)</code> if <code>n</code> was propagated to <code>n'</code>
     *         with edge <code>e</code>.
     */
    def propagate( n: N ): Option[(N, Option[E])] = Some((n, None))

    def init( args: Iterable[N] ) = {
        openList.clear; closedList.clear; seenList.clear; prunedList.clear
        predecessor.clear; distance.clear; edges.clear
        tDiscovery.clear; tClosed.clear
        goalList = Nil
        n_added = 0; n_opened = 0; n_pruned = 0
        tNextDiscovery = 0; tNextClosed = 0
        args.foreach( n => {
            distance.put(n, 0);
            openList.addOne((f(n), n))
            seenList.add(n)
            tDiscovery.put(n, this.getDiscoveryTime)
        })
    }

    def step( n: N ): Num = {
        closedList.add(n)
        this.n_closed += 1
        this.tClosed.put(n, this.getClosedTime)
        this.propagate(n) match {
            case None => { // prune n
                prunedList.add(n)
                tClosed.put(n, this.getClosedTime)
                addPrunedReason(n, "Propagation")
                n_pruned += 1
            }
            case Some((nProp, e)) => {
                e match { // add search move forced by propagation
                    case Some(edge) => {
                        predecessor.put(nProp, n)
                        edges.put(nProp, edge)
                        distance.put(nProp, distance(n))
                        closedList.add(nProp)
                        tDiscovery.put(nProp, this.getDiscoveryTime)
                        tClosed.put(nProp, this.getClosedTime)
                    }
                    case None => {}
                }

                val expansion = expand(nProp)
                this.n_opened += expansion.size
                logger.info(s"Expansion size: ${expansion.size}.")
                logger.depth += 1
                for ((edge, dest) <- expansion if !seenList.contains(dest)) {
                    seenList.add(dest)
                    predecessor.put(dest, nProp)
                    edges.put(dest, edge)
                    distance.put(dest, distance(nProp) + 1)
                    this.tDiscovery.put(dest, getDiscoveryTime)
                    var pruneFunction: Option[N => Boolean] = None
                    val isPruned = this.pruneFunctions.exists(f => {
                        val answer = f(dest)
                        if ( answer ) pruneFunction = Some(f)
                        answer
                    })
                    if (isPruned) {
                        prunedList.add(dest)
                        tClosed.put(dest, this.getClosedTime)
                        addPrunedReason(dest, pruneFunction.get.getClass.getSimpleName)
                        n_pruned += 1
                    } else {
                        val fVal = f(dest)
                        if ( pruneOnInfiniteHeuristicValue && fVal.isInfPos ) {
                            prunedList.add(dest)
                            tClosed.put(dest, this.getClosedTime)
                            addPrunedReason(dest, "h=+INF")
                            logger.info(s"Node pruned because heuristic value is infinite")
                            n_pruned += 1
                        } else {
                            logger.info(s"Node score f: $fVal")
                            logger.fine(s"Edge: $edge")
                            logger.finer(s"  Path:: ${pathTo(dest).mkString(" <- ")}")
                            openList.addOne((fVal, dest))
                            n_added += 1
                        }

                    }
                }
                logger.depth -= 1
                logger.info(s"Added: $n_added, pruned: $n_pruned, opened: $n_opened")
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
        logger.info(s"Next from ${openList.size} choices")
        if (openList.isEmpty) None
        else {
            val node = openList.dequeue._2
            val goalReached = isGoal(node)
            logger.info(s"Selected node is goal: $goalReached")
            Some((node, goalReached))
        }
    }

    def path(n: N): List[E] = pathTo(n).reverse

    @tailrec
    final def search: Option[Seq[E]] = {
        next match {
            case None => None
            case Some((n, true)) => {
                this.goalList = n :: this.goalList
                Some(pathTo(n).reverse)
            }
            case Some((n, false)) => { step(n); search }
        }
    }

    private def pathTo( dest: N ): List[E] = {
        if (!edges.isDefinedAt(dest)) Nil
        else edges(dest) :: pathTo(predecessor(dest))
    }
}
