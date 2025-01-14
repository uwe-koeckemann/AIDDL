package org.aiddl.common.scala.search

import org.aiddl.common.scala.math.graph.Graph2Dot
import org.aiddl.common.scala.math.graph.GraphType.Directed
import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.representation.*

import scala.annotation.tailrec
import scala.collection.mutable
import scala.collection.mutable.{HashMap, HashSet, PriorityQueue}

trait GenericGraphSearch[E, N] extends Verbose {

    private var omegas: Vector[Num] = Vector.empty
    protected var heuristics: Vector[Heuristic[N]] = Vector.empty
    var pruneFunctions: List[N => Boolean] = Nil
    var pruneOnInfiniteHeuristicValue = true
    var bestSolutionCost: Num = InfPos()


    private var openLists: Vector[mutable.PriorityQueue[(Num, N)]] =
        Vector(new mutable.PriorityQueue[(Num, N)]()(Ordering.by((x, _) => -x )))
    protected var currentHeuristicIndex: Int = 0

    val closedList = new mutable.HashSet[N]
    val seenList = new mutable.HashSet[N]
    val prunedList = new mutable.HashSet[N]
    val prunedReason = new mutable.HashMap[N, String]
    var goalList: List[N] = Nil
    var solutions: List[Seq[E]] = Nil

    val predecessor = new mutable.HashMap[N, N]
    val predecessors = new mutable.HashMap[N, List[N]]
    val inEdge = new mutable.HashMap[N, E]
    val inEdges = new mutable.HashMap[N, List[E]]

    val distance = new mutable.HashMap[N, Num]
    val heuristicValueCache = new mutable.HashMap[(Int, N), Num]()

    private val tDiscovery = new mutable.HashMap[N, Int]
    private val tClosed = new mutable.HashMap[N, Int]

    var n_added = 0
    var n_opened = 0
    var n_closed = 0
    var n_pruned = 0

    private var tNextDiscovery: Int = 0
    private var tNextClosed: Int = 0

    def isGoal(node: N): Boolean
    def expand(node: N): Seq[(E, N)]
    def cost(edge: E): Num = Num(1)

    /**
     * Propagate a node. This may lead to pruning, no change, or a forced move in the search space.
     * @param node
     * @return <code>None</code> if propagation finds node inconsistent, <code>Some(n, None)</code> if no change was
     *         made to <code>n</code>, <code>Some(n', e)</code> if <code>n</code> was propagated to <code>n'</code>
     *         with edge <code>e</code>.
     */
    def propagate(node: N ): Option[(N, Option[E])] = Some((node, None))

    def addHeuristic(omega: Num, heuristic: Heuristic[N]): Unit = {
        assert(Num(0) <= omega && omega <= Num(1))

        if this.heuristics.nonEmpty
        then this.openLists =
            this.openLists.appended(new mutable.PriorityQueue[(Num, N)]()(Ordering.by((x, _) => -x)))

        this.omegas = this.omegas.appended(omega)
        this.heuristics = this.heuristics.appended(heuristic)
    }

    def init( args: Iterable[N] ): Unit = {
        closedList.clear; seenList.clear; prunedList.clear
        this.openLists.indices.foreach(i => openLists(i).clear())
        this.currentHeuristicIndex = 0
        predecessor.clear; distance.clear; inEdge.clear
        predecessors.clear(); inEdges.clear
        tDiscovery.clear; tClosed.clear
        goalList = Nil
        n_added = 0; n_opened = 0; n_pruned = 0
        tNextDiscovery = 0; tNextClosed = 0
        args.foreach( n => {
            distance.put(n,Num(0));
            openLists.indices.foreach(i => {
                openLists(i).addOne((fValue(i, n), n))
            })
            seenList.add(n)
            tDiscovery.put(n, this.getDiscoveryTime)
        })
    }

    def step( n: N ): Num = {
        closedList.add(n)
        this.n_closed += 1
        this.tClosed.put(n, this.getClosedTime)

        val expansion =
            expand(n)
              .filterNot((_, expNode) => prunedList contains expNode)

        logger.info(s"Expansion size: ${expansion.size}.")
        logger.depth += 1

        for ( (expEdge, expNode) <- expansion ) {
            var addToOpen = false
            if !seenList.contains(expNode) || distance(expNode) > distance(n) + cost(expEdge)
            then {
                n_opened += 1
                if !seenList.contains(expNode) then {
                    this.tDiscovery.put(expNode, getDiscoveryTime)
                }
                this.addToSearchSpace(n, expEdge, expNode)
                addToOpen = true
            }
            else if seenList.contains(expNode) && distance(expNode) == distance(n) + cost(expEdge)
            then { // Keep track of alternative paths
                predecessors.put(expNode, n :: predecessors(expNode))
                inEdges.put(expNode, expEdge :: inEdges(expNode))
            }

            val propResult = this.propagate(expNode)
            val (isPrunedByPropagation, propNode) = {
                propResult match {
                    case None => {
                        addPrunedReason(n, "Propagation")
                        (true, expNode)
                    }
                    case Some((propNode, edgeOption)) => {
                        edgeOption.foreach(edge => {
                            this.addToSearchSpace(expNode, edge, propNode)
                            closedList.add(expNode)
                            tDiscovery.put(propNode, this.getDiscoveryTime)
                            tClosed.put(propNode, this.getClosedTime)
                        })
                        (false, propNode)
                    }
                }
            }

            if isPrunedByPropagation || isPrunedByInfiniteHeuristicValue(propNode) || isPrunedByFunction(propNode) then {
                prunedList.add(propNode)
                prunedList.add(expNode)
                tClosed.put(propNode, this.getClosedTime)
                logger.info(s"Node pruned because of: ${this.prunedReason(propNode)} ")
                n_pruned = prunedList.size
            } else {
                val fVal = fValue(currentHeuristicIndex, propNode)
                logger.info(s"Node score f: $fVal")
                logger.fine(s"Edge: $expEdge")
                logger.finer(s"  Path:: ${pathTo(propNode).mkString(" <- ")}")

                if addToOpen then {
                    openLists.indices.foreach(i => {
                        openLists(i).addOne((fValue(i, propNode), propNode))
                    })
                }
                n_added += 1
            }
        }
        logger.depth -= 1
        this.currentHeuristicIndex = (this.currentHeuristicIndex + 1) % this.openLists.size
        Num(n_added)
    }

    private def addToSearchSpace(from: N, edge: E, to: N): Unit = {
        predecessor.put(to, from)
        predecessors.put(to, List(from))
        inEdge.put(to, edge)
        inEdges.put(to, List(edge))
        distance.put(to, distance(from) + cost(edge))
        seenList.add(to)
    }

    private def isPrunedByFunction(node: N): Boolean = {
        this.pruneFunctions.exists(f => {
            val answer = f(node)
            if (answer) {
                addPrunedReason(node, f.getClass.getSimpleName)
            }
            answer
        })
    }

    private def isPrunedByInfiniteHeuristicValue(node: N): Boolean = {
        val fValue = this.fValue(currentHeuristicIndex, node)
        val answer = pruneOnInfiniteHeuristicValue && fValue.isInfPos
        if answer then {
            addPrunedReason(node, s"h=+INF")
        }
        answer
    }

    private def addPrunedReason(node: N, reason: String) = {
        this.prunedReason.getOrElseUpdate(node, reason)
    }

    private def fValue(i: Int, n: N) = {
        if omegas.isEmpty
        then distance(n)
        else if omegas(i) < Num(1) then
            heuristicValue(i, n) * omegas(i) + distance(n) * (Num(1.0) - omegas(i))
        else
            heuristicValue(i, n)
    }

    private def heuristicValue(i: Int, n: N): Num = {
        this.heuristicValueCache.getOrElseUpdate((i, n), {
            if this.heuristics.isEmpty
            then Num(0)
            else this.heuristics(i)(n)
        })
    }

    def next: Option[(N, Boolean)] = {
        this.dropWhileClosedOrPrunedFromCurrentQueue()
        logger.info(s"Next from ${openLists(currentHeuristicIndex).size} choices")
        if (openLists(currentHeuristicIndex).isEmpty) None
        else {
            val node = openLists(currentHeuristicIndex).dequeue._2
            val goalReached = isGoal(node)
            logger.info(s"Selected node: $node (is goal? $goalReached)")
            Some((node, goalReached))
        }
    }

    def dropWhileClosedOrPrunedFromCurrentQueue(): Unit = {
        if this.openLists.size > 1 then {
            openLists(this.currentHeuristicIndex).dropWhile((_, node) => {
                this.closedList.contains(node) || this.prunedList.contains(node)
            })
        }
    }

    def path(n: N): List[E] =
        pathTo(n).reverse

    def nodePath(node: N): List[N] =
        nodePathTo(node).reverse

    @tailrec
    final def search: Option[Seq[E]] = {
        next match {
            case None => None
            case Some((n, true)) => {
                this.goalList = n :: this.goalList
                solutions = path(n) :: solutions
                if distance(n) < bestSolutionCost then {
                    bestSolutionCost = distance(n)
                }
                Some(path(n))
            }
            case Some((n, false)) => { step(n); search }
        }
    }

    private def pathTo( dest: N ): List[E] = {
        if (!inEdge.isDefinedAt(dest)) Nil
        else inEdge(dest) :: pathTo(predecessor(dest))
    }

    private def nodePathTo(dest: N): List[N] = {
        if !predecessor.isDefinedAt(dest)
        then List(dest)
        else predecessor(dest) :: nodePathTo(predecessor(dest))
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
                        if this.inEdge(node).isInstanceOf[Reasoned] then
                            this.inEdge(node).asInstanceOf[Reasoned].reasonStr
                        else this.inEdge(node).toString
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
                    if ( this.inEdge(node).isInstanceOf[Term] ) this.inEdge(node).asInstanceOf[Term]
                    else Str(this.inEdge(node).toString)

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

    private def getDiscoveryTime: Int = {
        tNextDiscovery += 1
        tNextDiscovery
    }

    private def getClosedTime: Int = {
        tNextClosed += 1
        tNextClosed
    }
}
