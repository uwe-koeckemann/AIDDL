package org.aiddl.common.scala.search

import org.aiddl.common.scala.Common.NIL
import org.aiddl.core.scala.function.{Function, Initializable, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.logger.Logger

import scala.annotation.tailrec
import scala.collection.mutable.{HashMap, HashSet}

trait TreeSearch extends Function with Initializable with Verbose {
    var cDeadEnd = 0
    var cConsistentNodes = 0

    /** Allow to prune incomplete branches with the costAcceptable method. */
    var allowEarlyCostPruning = false

    var choice: List[Term]  = Nil
    var searchSpace: List[Seq[Term]] = Nil
    var searchIdx: List[Int] = Nil

    var solution: Option[List[Term]] = None
    var best: Num = InfPos()
    var failed = false

    val Expand = Sym("expand")
    val Next = Sym("next")

    val loggerName = "TreeSearch"

    def expand: Option[Seq[Term]]
    def isConsistent: Boolean = true
    def cost( choice: List[Term] ): Option[Num] = None

    def choiceHook: Unit = ()
    def expandHook: Unit = ()
    def backtrackHook: Unit = ()
    def solutionFoundHook: Unit = ()

    def assembleSolution( choice: List[Term] ): Option[List[Term]] = Some(choice)

    def cost: Option[Num] = cost(choice)
    def costAcceptable(c: Num): Boolean = c < best

    def init( args: Term ) = {
        choice = Nil
        searchSpace = Nil
        searchIdx = Nil
        solution = None
        best = InfPos()
        failed = false;
    }

    def apply( args: Term ): Term =
        args match {
            case Tuple(Sym("search")) => search match { case Some(c) => ListTerm(c) case None => NIL }
            case Tuple(Sym("optimal")) => optimal match { case Some(c) => ListTerm(c) case None => NIL }
            case _ => {
                init(args)
                search match {
                    case Some(s) => ListTerm(s)
                    case None => NIL
                }
            }
        }

    @tailrec
    final def optimal: Option[List[Term]] = {
        search match {
            case None => solution
            case _ => optimal
        }
    }

    @tailrec
    final def search: Option[List[Term]] = {
        if ( failed ) None
        else {
            logger.info(s"Expanding: $choice")
            expand match {
                case None =>
                    val isNewBest = (cost match {
                        case Some(c) =>
                            if (costAcceptable(c)) {
                                best = c
                                true
                            } else {
                                false
                            }
                        case None => true
                    })
                    if (isNewBest) {
                        solution = assembleSolution(choice)
                        logger.info(s"Solution: $solution (best=$best)")
                        solutionFoundHook
                    }

                    backtrack
                    solution
                case Some(exp) => {
                    logger.info(s"  Expansion: $exp")
                    searchSpace = exp :: searchSpace
                    searchIdx = -1 :: searchIdx
                    choice = Sym("NIL") :: choice
                    expandHook
                    if ( backtrack == None ) {
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
    final def backtrack: Option[List[Term]] = {
        logger.info(s"Backtracking: $choice")
        searchIdx = searchIdx.dropWhile( idx => {
            val noChoice = idx+1 >= searchSpace.head.size;
            if (noChoice) {
                searchSpace = searchSpace.tail;
                choice = choice.tail
                backtrackHook
            }
            noChoice
        })
        if (searchSpace.isEmpty) {
            failed = true
            None
        } else {
            val idx = searchIdx.head + 1
            searchIdx = idx :: searchIdx.tail
            choice = searchSpace.head(idx) :: choice.tail
            choiceHook
            if ( isConsistent
                && {!allowEarlyCostPruning || (cost match { case Some(c) => costAcceptable(c) case None => true })} ) {
                cConsistentNodes += 1
                Some(choice)
            } else {
                logger.info(s"Rejected (SAT=$isConsistent, COST=$cost, BEST=$best) : $choice")
                cDeadEnd += 1
                backtrack
            }
        }
    }
}