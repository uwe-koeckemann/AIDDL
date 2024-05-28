package org.aiddl.common.scala.search

import org.aiddl.common.scala.math.graph.Graph2Dot
import org.aiddl.common.scala.math.graph.GraphType.Directed
import org.aiddl.core.scala.function.{Function, Initializable, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.StopWatch
import org.aiddl.core.scala.util.logger.Logger

import scala.annotation.tailrec
import scala.collection.mutable
import scala.collection.mutable.{HashMap, HashSet, PriorityQueue}

trait GenericGraphSearch[E, N] extends Verbose {
    //val openList = new PriorityQueue[(Num, N)]()(Ordering.by( (x, _) => -x ))
    //var omega = Num(0.5)

    //protected var heuristics: Vector[N => Num] = Vector.empty
    protected var omegas: Vector[Num] = Vector.empty // Vector(this.omega)
    protected var openLists: Vector[PriorityQueue[(Num, N)]] = Vector.empty // Vector(openList)
    protected var i_h = 0

    def addHeuristic(omega: Num): Unit = {
        assert(Num(0) <= omega && omega <= Num(1))
        //this.heuristics = this.heuristics.appended(h)
        this.omegas = this.omegas.appended(omega)
        this.openLists = this.openLists.appended(new PriorityQueue[(Num, N)]()(Ordering.by( (x, _) => -x )))
    }

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

    def h( i: Int, n: N ): Num
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
        //openList.clear;
        closedList.clear; seenList.clear; prunedList.clear
        (0 until this.openLists.size).foreach(i => openLists(i).clear())
        this.i_h = 0
        predecessor.clear; distance.clear; edges.clear
        tDiscovery.clear; tClosed.clear
        goalList = Nil
        n_added = 0; n_opened = 0; n_pruned = 0
        tNextDiscovery = 0; tNextClosed = 0
        args.foreach( n => {
            distance.put(n, 0);
            (0 until openLists.length).foreach( i => {
                openLists(i).addOne((f(i, n), n))
            })
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
                        val fVal = f(i_h, dest)
                        if ( pruneOnInfiniteHeuristicValue && fVal.isInfPos ) {
                            prunedList.add(dest)
                            tClosed.put(dest, this.getClosedTime)
                            addPrunedReason(dest, s"h=+INF")
                            logger.info(s"Node pruned because heuristic value is infinite")
                            n_pruned += 1
                        } else {
                            logger.info(s"Node score f: $fVal")
                            logger.fine(s"Edge: $edge")
                            logger.finer(s"  Path:: ${pathTo(dest).mkString(" <- ")}")
                            openLists.indices.foreach(i => {
                                if i == i_h then {
                                    openLists(i).addOne((fVal, dest))
                                } else {
                                    openLists(i).addOne((f(i, dest), dest))
                                }
                            })

                            n_added += 1
                        }
                    }
                }
                logger.depth -= 1
                logger.info(s"Added: $n_added, pruned: $n_pruned, opened: $n_opened")
            }
        }

        this.i_h = (this.i_h + 1) % this.openLists.size
        if this.openLists.size > 1 then { // make sure head of next open list is not closed or pruned
            while (!openLists(this.i_h).isEmpty
              && (this.closedList.contains(openLists(this.i_h).head._2)
              || this.prunedList.contains(openLists(this.i_h).head._2)))  {
                openLists(this.i_h).dequeue
            }
        }
        Num(n_added)
    }

    private def f(i: Int, n: N) = {
        if openLists.isEmpty then Num(0)
        else if omegas(i) < Num(1) then
            h(i, n) * omegas(i) + g(n) * (Num(1.0) - omegas(i))
        else
            h(i, n)
    }

    def g(n: N): Num = Num(distance(n))

    def next: Option[(N, Boolean)] = {
        logger.info(s"Next from ${openLists(i_h).size} choices")
        if (openLists(i_h).isEmpty) None
        else {
            val node = openLists(i_h).dequeue._2
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
                Some(path(n))
            }
            case Some((n, false)) => { step(n); search }
        }
    }

    private def pathTo( dest: N ): List[E] = {
        if (!edges.isDefinedAt(dest)) Nil
        else edges(dest) :: pathTo(predecessor(dest))
    }

    def searchGraph2File(name: String): Unit = {
        val gt2 = new Graph2Dot(Directed)
        gt2.graph2file(this.graph, name)
    }

    def graph: Term = {
        var id: Long = 0L
        var nodeIds: Map[N, Term] = Map.empty
        var nodes: Set[Term] = Set.empty
        var edges: Set[Term] = Set.empty
        var nodeContent: Map[Term, N] = Map.empty
        var nodeAttributes: Map[Term, Set[Term]] = Map.empty.withDefaultValue(Set.empty)
        var edgeAttributes: Map[Term, Set[Term]] = Map.empty.withDefaultValue(Set.empty)

        var edgeLabels: Set[Term] = Set.empty

        var done: Set[N] = Set.empty

        def processNode(node: N, shape: Sym, style: Sym): Term = {
            if (!done(node)) {
                done = done + node
                val nodeId = nodeIds.getOrElse(node, {
                    id += 1
                    Sym(s"n$id")
                    Str(s"${this.tDiscovery(node)}/${this.tClosed.getOrElse(node, "-")}")
                })

                nodes = nodes + nodeId
                nodeContent = nodeContent.updated(nodeId, node)
                nodeIds = nodeIds.updated(node, nodeId)

                if (this.predecessor.contains(node)) {
                    val preNode = this.predecessor(node)
                    val preNodeId = nodeIds.getOrElse(preNode, {
                        id += 1
                        Sym(s"n$id")
                        Str(s"${this.tDiscovery(preNode)}/${this.tClosed.getOrElse(preNode, "-")}")
                    })
                    nodeIds = nodeIds.updated(preNode, preNodeId)
                    val edge = Tuple(preNodeId, nodeId)
                    edges += edge

                    val reason =
                        if this.edges(node).isInstanceOf[Reasoned] then
                            this.edges(node).asInstanceOf[Reasoned].reasonStr
                        else this.edges(node).toString
                    if (!reason.isEmpty) {
                        edgeLabels = edgeLabels + KeyVal(edge, Str(reason))
                    }
                }

                val nodeTerm =
                    if ( node.isInstanceOf[Term] ) node.asInstanceOf[Term]
                    else Str(node.toString)

                val currentAtts = nodeAttributes(nodeId)
                nodeAttributes = nodeAttributes.updated(nodeId, currentAtts ++ Set(KeyVal(Sym("shape"), shape), KeyVal(Sym("style"), style), KeyVal(Sym("content"), nodeTerm)))
                nodeId
            } else {
                nodeIds(node)
            }
        }

        for (node <- this.prunedList) {
            val id = processNode(node, Sym("box"), Sym("filled"))
            val reasonNodeId = Tuple(Sym("reason"), id)
            val edge = Tuple(id, reasonNodeId)
            nodes = nodes + reasonNodeId
            edges = edges + edge
            nodeAttributes = nodeAttributes.updated(reasonNodeId, Set(
                KeyVal(Sym("shape"), Sym("note")),
                KeyVal(Sym("label"), Str(this.prunedReason(node)))
            ))

        }
        for (node <- this.closedList) {
            processNode(node, Sym("circle"), Sym("solid"))
        }
        for ((_, node) <- this.openLists(0)) {
            processNode(node, Sym("circle"), Sym("dotted"))
        }
        for (node <- this.goalList) {
            processNode(node, Sym("circle"), Sym("filled"))
            var curNode = node
            var preNode = this.predecessor.get(node)
            while {
                preNode != None
            } do {
                val edge = Tuple(nodeIds(preNode.get), nodeIds(curNode))
                val edgeContent =
                    if ( this.edges(node).isInstanceOf[Term] ) this.edges(node).asInstanceOf[Term]
                    else Str(this.edges(node).toString)

                val currentAtts = edgeAttributes(edge)
                edgeAttributes = edgeAttributes.updated(edge, currentAtts ++ Set(KeyVal(Sym("style"), Sym("dashed")), KeyVal(Sym("content"), edgeContent)))

                curNode = preNode.get
                preNode = predecessor.get(curNode)
            }
        }
        val nodeAttsTerm = SetTerm(nodeAttributes.map((k, v) => KeyVal(k, SetTerm(v))).toSet)
        val edgeAttsTerm = SetTerm(edgeAttributes.map((k, v) => KeyVal(k, SetTerm(v))).toSet)


        ListTerm(
            KeyVal(Sym("V"), SetTerm(nodes)),
            KeyVal(Sym("E"), SetTerm(edges)),
            KeyVal(Sym("node-attributes"), nodeAttsTerm),
            KeyVal(Sym("edge-attributes"), edgeAttsTerm),
            KeyVal(Sym("labels"), SetTerm(edgeLabels))
        )
    }
}
