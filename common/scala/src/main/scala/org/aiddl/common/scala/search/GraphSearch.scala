package org.aiddl.common.scala.search

import scala.annotation.tailrec
import scala.collection.mutable.HashMap
import scala.collection.mutable.HashSet
import scala.collection.mutable.PriorityQueue
import org.aiddl.core.scala.function.Function
import org.aiddl.core.scala.function.Initializable
import org.aiddl.core.scala.function.Verbose
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.Common.NIL
import org.aiddl.core.scala.util.logger.Logger
import org.aiddl.core.scala.util.StopWatch

trait GraphSearch extends Function with Initializable with Verbose {
    val openList = new PriorityQueue[(Num, Term)]()(Ordering.by( (x, y) => -x ))
    val closedList = new HashSet[Term]
    val seenList = new HashSet[Term]

    val predecessor = new HashMap[Term, Term]
    val distance = new HashMap[Term, Int]
    val edges = new HashMap[Term, Term]

    var pruneFunctions: List[Function] = Nil

    var n_added = 0
    var n_opened = 0
    var n_pruned = 0
    var includePathLength = false
    var omega = Num(0.5)


    def h( n: Term ): Num
    def isGoal( n: Term ): Boolean
    def expand( n: Term ): Seq[Term]

    def apply( args: Term ): Term = {
        args match {
            case Tuple(Sym("search")) => search
            case Tuple(Sym("expand"), source) => step(source)
            case Tuple(Sym("next")) => next match { case (n, r) =>  Tuple(KeyVal(Sym("node"), n), Sym("is-goal"), Bool(r)) }
            case Tuple(Sym("is-closed"), node) => { Bool(closedList.contains(node)) }
            case Tuple(Sym("get"), Sym("num-added")) => { Num(n_added) }
            case Tuple(Sym("get"), Sym("num-opened")) => { Num(n_opened) }
            case Tuple(Sym("get"), Sym("num-pruned")) => { Num(n_pruned) }
            case Tuple(Sym("get"), Sym("size-open")) => { Num(openList.size) }
            case Tuple(Sym("get"), Sym("size-closed")) => { Num(closedList.size) }
            case Tuple(Sym("get"), Sym("size-seen")) => { Num(seenList.size) }
            case Tuple(Sym("get"), Sym("distance"), node) => { Num(distance(node)) }
            case Tuple(Sym("get"), Sym("path"), node) => {  ListTerm(pathTo(node).reverse) }
            case _ => ???
        }
    }

    def init( args: Term ) = {
        openList.clear; closedList.clear; seenList.clear
        predecessor.clear; distance.clear; edges.clear
        n_added = 0; n_opened = 0; n_pruned = 0
        args.asCol.foreach( n => {
            distance.put(n, 0);
            openList.addOne((f(n).asNum, n))
            seenList.add(n) })
    }

    def step( n: Term ): Num = {
        closedList.add(n)
        val expansion = expand(n)
        this.n_opened += expansion.size
        for ( Tuple(edge, dest) <- expansion if !seenList.contains(dest) ) {
            seenList.add(dest)
            val isPruned = this.pruneFunctions.exists( f => f(dest).boolVal )
            if (isPruned) { n_pruned += 1 }
            else {
                n_added += 1
                predecessor.put(dest, n)
                edges.put(dest, edge)
                distance.put(dest, distance(n) + 1)
                val fVal = f(dest)
                logger.info(s"Node score f: $fVal")
                logger.fine(s"  Path:: ${pathTo(dest).mkString(" <- ")}")
                openList.addOne((fVal, dest))
            }
        }
        Num(n_added)
    }

    def g(n: Term): Num = Num(distance(n))
    def f(n: Term): Num =
        if ( includePathLength )
            h(n)*omega + g(n)*(Num(1.0)-omega)
        else
            h(n)

    def next: (Term, Boolean) = {
        if (openList.isEmpty) (NIL, false)
        else {
            val succ = openList.dequeue._2
            (succ, isGoal(succ))
        }
    }

    def path(n: Term): ListTerm = ListTerm(pathTo(n).reverse)

    @tailrec
    final def search: Term = {
        next match {
            case (NIL, _) => NIL
            case (n, true) => ListTerm(pathTo(n).reverse)
            case (n, false) => { step(n); search }
        }
    }

    private def pathTo( dest: Term ): List[Term] = {
        if (!edges.isDefinedAt(dest)) Nil
        else edges(dest) :: pathTo(predecessor(dest))
    }
}
