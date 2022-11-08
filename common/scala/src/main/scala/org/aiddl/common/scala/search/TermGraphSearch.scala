package org.aiddl.common.scala.search

import org.aiddl.common.scala.Common.NIL
import org.aiddl.core.scala.function.{Function, Initializable, Verbose}
import org.aiddl.core.scala.representation.*
import org.aiddl.core.scala.util.logger.Logger
import org.aiddl.core.scala.util.StopWatch

import scala.annotation.tailrec
import scala.collection.mutable.{HashMap, HashSet, PriorityQueue}

trait TermGraphSearch extends GenericGraphSearch[Term, Term] with Function with Initializable with Verbose {
    def init( args: Term ) = super.init(args.asList.list)

    def apply( args: Term ): Term = {
        args match {
            case Tuple(Sym("search")) => search match {
                case None => NIL
                case Some(path) => ListTerm(path)
            }
            case Tuple(Sym("expand"), source) => step(source)
            case Tuple(Sym("next")) => next match {
                case None => NIL
                case Some((n, r)) =>  Tuple(KeyVal(Sym("node"), n), KeyVal(Sym("is-goal"), Bool(r)))
            }
            case Tuple(Sym("is-closed"), node) => { Bool(closedList.contains(node)) }
            case Tuple(Sym("get"), Sym("num-added")) => Num(n_added)
            case Tuple(Sym("get"), Sym("num-opened")) => Num(n_opened)
            case Tuple(Sym("get"), Sym("num-pruned")) => Num(n_pruned)
            case Tuple(Sym("get"), Sym("size-open")) => Num(openList.size)
            case Tuple(Sym("get"), Sym("size-closed")) => Num(closedList.size)
            case Tuple(Sym("get"), Sym("size-seen")) => Num(seenList.size)
            case Tuple(Sym("get"), Sym("distance"), node) => Num(distance(node))
            case Tuple(Sym("get"), Sym("path"), node) => ListTerm(path(node))
            case _ => ???
        }
    }

    def addPruningFunction( f: Function ) = {
        this.pruneFunctions = (x => f(x).asBool.v) :: this.pruneFunctions
    }
}
