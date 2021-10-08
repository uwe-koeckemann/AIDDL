package org.aiddl.common.scala.reasoning.logic.propositional

import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.search.TreeSearch

import org.aiddl.core.scala.representation.TermImplicits._
import org.aiddl.core.scala.representation.TermCollectionImplicits.term2ListTerm

class DpllSolver extends TreeSearch {
    override def init( args: Term ) = {
        this.prs = List(SetTerm(
            KeyVal(Sym("a"), ListTerm.empty),
            KeyVal(Sym("phi"), args),
            KeyVal(Sym("open"), SetTerm(args.flatMap( c => c.map( l => l.abs) ).toSet)) ))
        super.init(args)
    }

    private def a = prs.head(Sym("a"))
    private def Phi = prs.head(Sym("phi"))
    private def Open: SetTerm = prs.head(Sym("open")).asSet

    override def assembleSolution( c: List[Term] ): Option[List[Term]] = Some(c ++ a) 

    override def choose: Option[Term] = 
        if ( Open.isEmpty ) None else Some(ListTerm(-Open.head, Open.head))
        
    override def propagate: Option[Term] = {
        var phi = Phi
        if ( !choice.isEmpty ) phi = unitPropagate(ListTerm(choice.head), phi)
        if (phi.exists( _.size == 0 )) None
        else {
            val unit = phi.withFilter( c => c.size == 1 )
                          .map( c => c(0) )
            if ( !unit.isEmpty )  phi = unitPropagate(ListTerm(unit), phi)
            val pure = phi.flatMap(c => c.filter(_ < 0))
                          .filter( x => phi.exists( c => c.contains(-x) ))
            if ( !pure.isEmpty ) phi = unitPropagate(ListTerm(pure), phi)
            if ( phi.exists( _.size == 0 )) None
            else {
                val prop = unit ++ pure
                var closed = prop.map(_.abs).toSet
                if ( !choice.isEmpty ) closed = closed + choice.head.abs
                Some(SetTerm(
                    KeyVal(Sym("a"), ListTerm(a ++ prop)),
                    KeyVal(Sym("phi"), phi),
                    KeyVal(Sym("open"), SetTerm(Open.filter( e => !closed.contains(e) ).toSet))))
            }
        }
    }

    private def unitPropagate( a: CollectionTerm, phi: ListTerm ): ListTerm = 
        ListTerm(phi.withFilter( c => !c.containsAny(a) )
                    .map( c => ListTerm(c.filter( x => !a.contains(-x) ).toSeq ).toSeq))
}