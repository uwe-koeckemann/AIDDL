package org.aiddl.common.scala.search

import org.aiddl.common.scala.Common.NIL
import org.aiddl.core.scala.function.{Function, Initializable, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2ListTerm
import org.aiddl.core.scala.representation.TermImplicits.*
import org.aiddl.core.scala.tools.Logger

import scala.annotation.tailrec
import scala.collection.mutable.{HashMap, HashSet}

trait GenericTreeSearch[T, S] extends Verbose {
    var cDeadEnd = 0
    var cConsistentNodes = 0

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

    val nil: T
    def expand: Option[Seq[T]]

    def isConsistent: Boolean = true
    def cost( choice: List[T] ): Option[Num] = None

    def choiceHook: Unit = ()
    def expandHook: Unit = ()
    def backtrackHook: Unit = ()

    def assembleSolution( choice: List[T] ): Option[S]

    def cost: Option[Num] = cost(choice)
    def costImproved(c: Num): Boolean = c < best

    def reset = {
        choice = Nil
        searchSpace = Nil
        searchIdx = Nil
        solution = None
        best = InfPos()
        depth = 0
        failed = false;
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
            log(1, s"Expanding: $choice")
            expand match {
                case None =>
                    cost match
                        case Some(c) =>
                            if ( costImproved(c) ) {
                                best = c
                                solution = assembleSolution(choice)
                            }
                        case None => solution = assembleSolution(choice)
                    log(1, s"Solution: $solution")
                    backtrack
                    solution
                case Some(exp) => {
                    log(1, s"  Expansion: $exp")
                    searchSpace = exp :: searchSpace
                    searchIdx = -1 :: searchIdx
                    choice = nil :: choice
                    depth += 1
                    expandHook
                    if ( backtrack == None ) {
                        log(1, s"  Done!")
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
        log(1, s"Backtracking: $choice")
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
            failed = true
            None
        } else {
            val idx = searchIdx.head + 1
            searchIdx = idx :: searchIdx.tail
            choice = searchSpace.head(idx) :: choice.tail
            choiceHook
            if ( isConsistent
                && {cost match { case Some(c) => costImproved(c) case None => true }} ) {
                cConsistentNodes += 1
                log(1, s"Backtracked to: $choice")
                Some(choice)
            } else {
                log(1, s"Rejected: $choice")
                cDeadEnd += 1
                backtrack
            }
        }
    }
}