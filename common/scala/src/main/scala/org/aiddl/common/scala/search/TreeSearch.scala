package org.aiddl.common.scala.search

import scala.annotation.tailrec
import scala.collection.mutable.HashMap
import scala.collection.mutable.HashSet

import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.function.Verbose

import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.Common.NIL

import org.aiddl.core.scala.representation.TermImplicits._
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2ListTerm
import org.aiddl.core.scala.tools.Logger

trait TreeSearch extends Function with Initializable with Verbose {
    var choice: List[Term]  = Nil
    var searchSpace: List[ListTerm]  = Nil
    var searchIdx: List[Int] = Nil
    var prs: List[Term] = Nil

    var solution: Option[List[Term]] = None
    var best: Num = InfPos()
    var failed = false

    val Expand = Sym("expand")
    val Next = Sym("next")

    val loggerName = "TreeSearch"

    def choose: Option[Term]

    def isConsistent: Boolean = true
    def cost( choice: List[Term] ): Option[Num] = None
    def propagate: Option[Term] = Some(NIL)

    def assembleSolution( choice: List[Term] ): Option[List[Term]] = Some(choice)

    def cost: Option[Num] = cost(choice)

    def init( args: Term ) = {
        choice = Nil
        searchSpace = Nil
        searchIdx = Nil
        solution = None
        best = InfPos()
        failed = false;
        prs = propagate match {
            case Some(result) => List(result)
            case None => { failed = true; Nil }
        }
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

    private def propagationConsistent: Boolean = {
        propagate match {
            case Some(result) => prs = result :: prs; true
            case None => false
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
            log(1, s"Expanding: $choice")
            choose match {
                case None => 
                    cost match 
                        case Some(c) => 
                            if ( c < best ) {
                                best = c
                                solution = assembleSolution(choice)
                            }
                        case None => solution = assembleSolution(choice)
                    log(1, s"Solution: $solution")
                    backtrack
                    solution
                case Some(c) => {
                    searchSpace = c :: searchSpace
                    searchIdx = -1 :: searchIdx
                    choice = Sym("NIL") :: choice      
                    if ( backtrack == None ) {
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
        log(1, s"Backtracking: $choice")
        searchIdx = searchIdx.dropWhile( idx => {
            val noChoice = idx+1 >= searchSpace.head.size; 
            if (noChoice) { 
                searchSpace = searchSpace.tail; 
                choice = choice.tail 
                prs = prs.tail
            } 
            noChoice
        })
        if (searchSpace.isEmpty) None
        else {
            val idx = searchIdx.head + 1
            searchIdx = idx :: searchIdx.tail
            choice = searchSpace.head(idx) :: choice.tail
            if ( isConsistent
                && propagationConsistent
                && {cost match { case Some(c) => c < best case None => true }} )
                Some(choice)
            else {
                log(1, s"Rejected: $choice")
                backtrack
            }
        }
    }
}