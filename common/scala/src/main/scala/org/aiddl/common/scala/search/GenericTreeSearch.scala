package org.aiddl.common.scala.search

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.math.graph.Graph2Dot
import org.aiddl.common.scala.math.graph.GraphType.Directed
import org.aiddl.common.scala.math.graph.Terms.{Attributes, EdgeAttributes, Edges, Labels, Nodes}
import org.aiddl.core.scala.function.{Function, Initializable, Verbose}
import org.aiddl.core.scala.representation
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.logger.Logger

import scala.annotation.tailrec
import scala.collection.mutable.{HashMap, HashSet}

trait GenericTreeSearch[T, S] extends Verbose with Iterator[S] {
    var cDeadEnd = 0
    var cConsistentNodes = 0

    /** Allow to prune incomplete branches with the costAcceptable method. */
    var allowEarlyCostPruning = false

    var choice: List[T]  = Nil
    var searchSpace: List[Seq[T]] = Nil
    var searchIdx: List[Int] = Nil
    var depth = 0

    var solution: Option[S] = None
    var best: Num = InfPos()
    var failed = false

    val Expand = Sym("expand")
    val Next = Sym("next")

    val loggerName = "TreeSearch"

    var traceFlag: Boolean = false
    private var traceNodes: List[Term] = Nil
    private var traceEdges: Map[Term, List[Term]] = Map.empty.withDefaultValue(Nil)
    private var traceChoices: Map[(Term, Term), T] = Map.empty
    private var traceNodeLabels: Map[Term, Term] = Map.empty
    private var traceNodeShapes: Map[Term, Term] = Map.empty
    private var traceNodeFill: Map[Term, Term] = Map.empty
    private var traceNodeStyle: Map[Term, Term] = Map.empty

    private def traceEdge(from: Term, choice: T, to: Term): Unit = {
        traceNodes = to :: traceNodes
        traceEdges = traceEdges.updated(from, to ::  traceEdges(from))
        traceChoices = traceChoices.updated((from, to), choice)
    }

    private def traceNodeId(searchIdx: List[Int]): Term =
        if searchIdx.isEmpty
        then Sym("root")
        else Sym(s"n${searchIdx.mkString("-")}")

    def node(choices: Seq[T]): Option[Term] =
        None

    val nil: T
    def expand: Option[Seq[T]]

    def isConsistent: Boolean =
        true

    def cost( choice: List[T] ): Option[Num] =
        None

    def choiceHook: Unit = ()
    def expandHook: Unit = ()
    def backtrackHook: Unit = ()
    def solutionFoundHook: Unit = ()

    def assembleSolution( choice: List[T] ): Option[S]

    def cost: Option[Num] = cost(choice)
    def costAcceptable(c: Num): Boolean = c < best

    def reset: Unit = {
        choice = Nil
        searchSpace = Nil
        searchIdx = Nil
        solution = None
        best = InfPos()
        depth = 0
        failed = false
        cDeadEnd = 0
        cConsistentNodes = 0
    }

    @tailrec
    final def optimal: Option[S] = {
        search match {
            case None => solution
            case _ => optimal
        }
    }

    @tailrec
    final def search: Option[S] = {
        if ( failed ) None
        else {
            logger.info(s"Expanding: $choice")
            expand match {
                case None =>
                    val isNewBest = cost match {
                        case Some(c) =>
                            if (costAcceptable(c)) {
                                best = c
                                true
                            } else false
                        case None => true
                    }
                    if ( isNewBest ) {
                        solution = assembleSolution(choice)
                        if ( traceFlag ) {
                            val solutionNodeId = this.traceNodeId(searchIdx)
                            this.traceNodeShapes = this.traceNodeShapes.updated(solutionNodeId, Sym("doubleoctagon"))
                            this.traceNodeStyle = this.traceNodeStyle.updated(solutionNodeId, Sym("filled"))
                            this.traceNodeFill = this.traceNodeFill.updated(solutionNodeId, Sym("lightgrey"))
                            val costStr = if cost(choice).isDefined then s"${cost(choice).get}" else ""
                            //val solutionStr = if solution.isDefined then solution.get.toString else ""
                            val currentLabel = this.traceNodeLabels.getOrElse(solutionNodeId, Sym("root"))
                            val newLabel =
                                if currentLabel == Str("")
                                then Str(costStr)
                                else Str(currentLabel.toString + s"\\n$costStr")
                            this.traceNodeLabels = this.traceNodeLabels.updated(solutionNodeId, newLabel)
                        }
                        logger.info(s"Solution: $solution")
                        solutionFoundHook
                    } else {
                        if (traceFlag) {
                            val rejectedNodeId = this.traceNodeId(searchIdx)
                            this.traceNodeShapes = this.traceNodeShapes.updated(rejectedNodeId, Sym("doublecircle"))
                            this.traceNodeStyle = this.traceNodeStyle.updated(rejectedNodeId, Sym("filled"))
                            this.traceNodeFill = this.traceNodeFill.updated(rejectedNodeId, Sym("lightgrey"))
                            this.traceNodeLabels = this.traceNodeLabels.updated(rejectedNodeId, Str("<"))
                        }
                    }
                    backtrack
                    solution
                case Some(exp) => {
                    logger.info(s"  Expansion: $exp")
                    searchSpace = exp :: searchSpace
                    searchIdx = -1 :: searchIdx
                    choice = nil :: choice
                    depth += 1
                    expandHook
                    if ( traceFlag ) {
                        val nodeFromId = this.traceNodeId(searchIdx.tail)
                        if (nodeFromId == Sym("root")) {
                            traceNodes = nodeFromId :: traceNodes
                            this.node(choice.tail) match {
                                case Some(label) => {
                                    this.traceNodeLabels = this.traceNodeLabels.updated(nodeFromId, label)
                                    this.traceNodeShapes = this.traceNodeShapes.updated(nodeFromId, Sym("ellipse"))
                                }
                                case None => {
                                    this.traceNodeLabels = this.traceNodeLabels.updated(nodeFromId, Str(""))
                                    this.traceNodeShapes = this.traceNodeShapes.updated(nodeFromId, Sym("point"))
                                }
                            }
                        }
                        exp.zipWithIndex.foreach( (c, idx) => {
                            val nodeToId = this.traceNodeId(idx :: searchIdx.tail)
                            this.traceEdge(nodeFromId, c, nodeToId)
                            this.node(c :: choice.tail) match {
                                case Some(label) => {
                                    this.traceNodeLabels = this.traceNodeLabels.updated(nodeToId, label)
                                    this.traceNodeShapes = this.traceNodeShapes.updated(nodeToId, Sym("ellipse"))
                                }
                                case None =>  {
                                    this.traceNodeLabels = this.traceNodeLabels.updated(nodeToId, Str(""))
                                    this.traceNodeShapes = this.traceNodeShapes.updated(nodeToId, Sym("point"))
                                }
                            }
                        })
                    }
                    if (backtrack.isEmpty) {
                        logger.info(s"  Done!")
                        failed = true
                        None
                    } else {
                        search
                    }
                }
            }
        }
    }

    @tailrec
    final def backtrack: Option[List[T]] = {
        logger.info(s"Backtracking: $choice")
        searchIdx = searchIdx.dropWhile( idx => {
            val noChoice = idx+1 >= searchSpace.head.size;
            if (noChoice) {
                searchSpace = searchSpace.tail;
                choice = choice.tail
                depth -= 1
                backtrackHook
            }
            noChoice
        })

        if (searchSpace.isEmpty) {
            logger.info("Search space exhausted")
            failed = true
            None
        } else {
            val idx = searchIdx.head + 1
            searchIdx = idx :: searchIdx.tail
            choice = searchSpace.head(idx) :: choice.tail
            choiceHook
            val consistent = isConsistent
            if ( consistent && (!allowEarlyCostPruning || cost.flatMap(c => Some(costAcceptable(c))).getOrElse(true)) ) {
                cConsistentNodes += 1
                logger.info(s"Backtracked to: $choice")
                Some(choice)
            } else {
                logger.info(s"Rejected: $choice")
                cDeadEnd += 1

                if (traceFlag) {
                    val rejectedNodeId = this.traceNodeId(searchIdx)
                    this.traceNodeShapes = this.traceNodeShapes.updated(rejectedNodeId, Sym("doublecircle"))
                    this.traceNodeStyle = this.traceNodeStyle.updated(rejectedNodeId, Sym("filled"))
                    this.traceNodeFill = this.traceNodeFill.updated(rejectedNodeId, Sym("lightgrey"))
                    this.traceNodeLabels = this.traceNodeLabels.updated(rejectedNodeId, Str(if !consistent then "X" else "<"))
                }
                backtrack
            }
        }
    }

    def searchGraph2File(name: String): Unit = {
        val gt2 = new Graph2Dot(Directed)
        gt2.graph2file(this.graph, name)
    }

    def graph: Term = {
        var nodeAtts: Map[Term, Set[Term]] = Map.empty

        traceNodeLabels.foreach((node, label) => {
            nodeAtts = nodeAtts.updated(node, Set(KeyVal(Sym("label"), label)))})
        traceNodeShapes.foreach((node, shape) => nodeAtts = nodeAtts.updated(node, nodeAtts(node) + KeyVal(Sym("shape"), shape)))
        traceNodeStyle.foreach((node, style) => nodeAtts = nodeAtts.updated(node, nodeAtts(node) + KeyVal(Sym("style"), style)))
        traceNodeFill.foreach((node, color) => nodeAtts = nodeAtts.updated(node, nodeAtts(node) + KeyVal(Sym("fillcolor"), color)))

        ListTerm(
            KeyVal(Nodes, ListTerm(traceNodes.reverse)),
            KeyVal(Edges, ListTerm(traceEdges.flatMap( (source, targets) => targets.reverse.map( Tuple(source, _) )).toList)),
            KeyVal(Attributes, SetTerm(nodeAtts.map( (node, atts) => KeyVal(node, SetTerm(atts))).toSet)),
            KeyVal(Labels, SetTerm(traceChoices.map( (edge, label) => {
                val (a, b) = edge
                KeyVal(Tuple(a, b), Str(label.toString))
            }).toSet))
        )
    }

    override def hasNext: Boolean =
        if failed then false
        else {
            this.solution = this.search
            this.failed = this.failed || this.solution.isEmpty
            this.solution.isDefined
        }

    override def next(): S = {
        if this.solution.isEmpty
        then this.solution = this.search

        this.solution match {
            case Some(item) =>
                this.solution = None
                item
            case None =>
                throw new NoSuchElementException()
        }
    }
}