package org.aiddl.common.scala.reasoning.logic.propositional

import org.aiddl.common.scala.Common
import org.aiddl.core.scala.representation.*
import org.aiddl.common.scala.Common.NIL
import org.aiddl.common.scala.search.{GenericTreeSearch, TreeSearch}
import org.aiddl.core.scala.function.{Function, Initializable}
import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_Num
import org.aiddl.core.scala.representation.conversion.given_Conversion_Term_ListTerm

import scala.language.implicitConversions

class DpllSolver extends GenericTreeSearch[Term, List[Term]] with Function with Initializable {
    override def init( args: Term ) = {
        super.reset
        as = List(ListTerm.empty)
        Phis = List(args)
        OpenVars = List(SetTerm(args.asCol.flatMap( c => c.asCol.map( l => l.abs) ).toSet))
        isConsistent
    }

    override def node(choices: Seq[Term]): Option[Term] =
        Some(ListTerm(
            Phis.reverse.head.asList
              .filterNot(clause =>
                clause.asList.containsAny(ListTerm(choices)) || clause.asList.containsAny(this.a))
              .map(clause => ListTerm(clause.asList.filterNot( literal => choices.contains(-literal) || a.contains(-literal) )))))

    var as: List[ListTerm] = Nil
    var Phis: List[Term] = Nil
    var OpenVars: List[SetTerm] = Nil

    private def a = as.head
    private def Phi = Phis.head
    private def Open: SetTerm = OpenVars.head

    override def assembleSolution( c: List[Term] ): Option[List[Term]] =
        Some(c ++ a)

    override def expand: Option[Seq[Term]] = {
        val unit = Phis.head.withFilter(c => c.asCol.size == 1).map(c => c(0)).toList
        val pure: List[Term] =
            Phis.head.flatMap(c => c.asCol.filter(_ < 0)).filter(x => !Phis.head.exists(c => c.asCol.contains(-x))).toList ++
              Phis.head.flatMap(c => c.asCol.filter(_ > 0)).filter(x => !Phis.head.exists(c => c.asCol.contains(-x))).toList
        logger.info(s"Unit literals: $unit")
        logger.info(s"Pure literals: $pure")
        if (unit.nonEmpty) {
            Some(ListTerm(unit.head))
        } else if (pure.nonEmpty) {
            Some(ListTerm(pure.head))
        } else {
            if ( Open.isEmpty ) None else Some(ListTerm(-Open.head, Open.head))
        }
    }

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
                logger.info(s"Unit literals: $unit")
                if (unit.nonEmpty) {
                    change = true
                    propLits = propLits ++ unit
                    phi = unitPropagate(ListTerm(unit), phi)
                }
                val pure: List[Term] =
                    phi.flatMap(c => c.asCol.filter(_ < 0)).filter(x => !phi.exists(c => c.asCol.contains(-x))).toList ++
                      phi.flatMap(c => c.asCol.filter(_ > 0)).filter(x => !phi.exists(c => c.asCol.contains(-x))).toList
                logger.info(s"Pure literals: $pure")
                if (pure.nonEmpty) {
                    change = true
                    propLits = propLits ++ pure
                    phi = unitPropagate(ListTerm(pure.toSet.toList), phi)
                }
            }
            logger.info(s"Phi after propagation: $phi")
            if ( phi.exists( _.asCol.size == 0 )) false
            else {
                var closed = propLits.map(_.abs).toSet
                if ( !choice.isEmpty ) closed = closed + choice.head.abs
                logger.info(s"  Propagated clauses: $phi")
                logger.info(s"  Propagated literals: $propLits")
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

    override val nil: Term = ListTerm.empty

    override def apply(x: Term): Term =
        this.init(x)
        this.search match
            case Some(value) => ListTerm(value)
            case None => Common.NIL
}