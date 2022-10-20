package org.aiddl.common.scala.reasoning.logic.propositional

import org.aiddl.core.scala.representation._

import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.search.TreeSearch

import Term.given_Conversion_Term_Num
import Term.given_Conversion_Term_ListTerm


class DpllSolver extends TreeSearch {
    override def init( args: Term ) = {
        super.init(args)
        as = List(ListTerm.empty)
        Phis = List(args)
        OpenVars = List(SetTerm(args.asCol.flatMap( c => c.asCol.map( l => l.abs) ).toSet))
        isConsistent
    }

    var as: List[ListTerm] = Nil
    var Phis: List[Term] = Nil
    var OpenVars: List[SetTerm] = Nil

    private def a = as.head
    private def Phi = Phis.head
    private def Open: SetTerm = OpenVars.head

    override def assembleSolution( c: List[Term] ): Option[List[Term]] = Some(c ++ a) 

    override def expand: Option[Seq[Term]] =
        if ( Open.isEmpty ) None else Some(ListTerm(-Open.head, Open.head))

    override def backtrackHook: Unit = {
        as = as.tail
        Phis = Phis.tail
        OpenVars = OpenVars.tail
    }
        
    override def isConsistent: Boolean = {
        var phi = Phi.asCol
        if ( !choice.isEmpty ) phi = unitPropagate(ListTerm(choice.head), phi.asList)
        if (phi.exists( _.asCol.size == 0 )) false
        else {
            var change = true
            var propLits: List[Term] = Nil
            while { change } do {
                change = false
                val unit = phi.withFilter(c => c.asCol.size == 1).map(c => c(0)).toList
                if (unit.nonEmpty) {
                    change = true
                    propLits = propLits ++ unit
                    phi = unitPropagate(ListTerm(unit), phi)
                }
                val pure = phi.flatMap(c => c.asCol.filter(_ < 0)).filter(x => phi.exists(c => c.asCol.contains(-x))).toList
                if (pure.nonEmpty) {
                    change = true
                    propLits = propLits ++ pure
                    phi = unitPropagate(ListTerm(pure), phi)
                }
            }
            if ( phi.exists( _.asCol.size == 0 )) false
            else {
                var closed = propLits.map(_.abs).toSet
                if ( !choice.isEmpty ) closed = closed + choice.head.abs
                log(1, s"  Propagated clauses: $phi")
                log(1, s"  Propagated literals: $propLits")
                as = ListTerm(propLits ++ a) :: as
                Phis = phi :: Phis
                OpenVars = SetTerm(Open.filter( e => !closed.contains(e) ).toSet) :: OpenVars
                true
            }
        }
    }

    private def unitPropagate( a: CollectionTerm, phi: ListTerm ): ListTerm = 
        ListTerm(phi.withFilter( c => !c.asCol.containsAny(a) )
                    .map( c => ListTerm(c.asCol.filter( x => !a.contains(-x) ).toList )))
}